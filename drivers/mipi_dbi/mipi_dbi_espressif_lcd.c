#define DT_DRV_COMPAT espressif_esp_mipi_dbi_lcd

#include <zephyr/kernel.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "hal/lcd_types.h"
#include "hal/lcd_ll.h"
#include "soc/lcd_cam_struct.h"
#include "esp_rom_gpio.h"
#include "esp_intr_alloc.h"
#include "soc/interrupts.h"

#include <zephyr/dt-bindings/clock/esp32s3_clock.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/esp32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#include <soc.h>
#include <soc/gpio_sig_map.h>
#include <hal/gdma_channel.h>
#include <hal/gpio_hal.h>
#include <hal/gpio_types.h>
#include <hal/gpio_ll.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mipi_dbi_espressif_lcd, CONFIG_MIPI_DBI_LOG_LEVEL);

/* gpio_io_config_t is now provided by the espressif HAL (hal/gpio_types.h) */

/* GPIO configuration dump utility using ESP-IDF HAL functions */
static void esp_gpio_dump_io_configuration(uint64_t io_bit_mask)
{
	LOG_INF("================IO DUMP Start================");

	/* Initialize GPIO HAL context */

	while (io_bit_mask) {
		uint32_t gpio_num = __builtin_ffsll(io_bit_mask) - 1;
		gpio_hal_context_t gpio_hal = {
			.dev = GPIO_HAL_GET_HW(0)
		};
		io_bit_mask &= ~(1ULL << gpio_num);

		/* Use HAL function to get GPIO configuration */
		gpio_io_config_t io_config = {0};
		gpio_hal_get_io_config(&gpio_hal, gpio_num, &io_config);

		/* Convert drive capability enum to readable string */
		const char *drive_str[] = {"0(weak)", "1(stronger)", "2(medium)", "3(strongest)" };
		const char *drive_cap_str = (io_config.drv < GPIO_DRIVE_CAP_MAX) ? drive_str[io_config.drv] : "invalid";

		LOG_INF("IO[%u] -", gpio_num);
		LOG_INF("  Pullup: %d, Pulldown: %d, DriveCap: %s", io_config.pu, io_config.pd, drive_cap_str);
		LOG_INF("  InputEn: %d, OutputEn: %d, OpenDrain: %d", io_config.ie, io_config.oe, io_config.od);
		LOG_INF("  FuncSel: %u (%s)", io_config.fun_sel, (io_config.fun_sel == PIN_FUNC_GPIO) ? "GPIO" : "IOMUX");
		LOG_INF("  OE_CtrlByPeriph: %d, OE_Inv: %d, SleepSel: %d", io_config.oe_ctrl_by_periph, io_config.oe_inv, io_config.slp_sel);
		LOG_INF("  GPIO Matrix SigOut ID: %u%s", io_config.sig_out, (io_config.sig_out == SIG_GPIO_OUT_IDX) ? " (simple GPIO output)" : "");

		/* Check input signal connections using HAL function */
		if (io_config.ie && io_config.fun_sel == PIN_FUNC_GPIO) {
			uint32_t connected_signals = 0;
			for (int i = 0; i < SIG_GPIO_OUT_IDX && i < 256; i++) {
				if (gpio_hal_get_in_signal_connected_io(&gpio_hal, i) == gpio_num) {
					if (connected_signals == 0) {
						LOG_INF("  GPIO Matrix SigIn ID: %d", i);
					} else {
						LOG_INF("                         %d", i);
					}
					connected_signals++;
				}
			}
			if (connected_signals == 0) {
				LOG_INF("  GPIO Matrix SigIn ID: (simple GPIO input)");
			}
		}
	}
	LOG_INF("=================IO DUMP End=================");
}

/* Portable conversion from gpio_dt_spec to absolute GPIO pin number for ESP32 */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(gpio1))
static inline uint32_t gpio_dt2abs(const struct gpio_dt_spec *spec)
{
	/* Check if this gpio_dt_spec is using gpio1 controller */
	if (spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio1))) {
		/* gpio1 pins need +32 offset for pins < 32 */
		return spec->pin + ((spec->pin < 32) ? 32 : 0);
	}
	/* gpio0 pins use pin number directly */
	return spec->pin;
}
#else
static inline uint32_t gpio_dt2abs(const struct gpio_dt_spec *spec)
{
	/* Only gpio0 available, pin number is absolute */
	return spec->pin;
}
#endif

/* The MIPI DBI spec allows 8, 9, and 16 bits */
#define MIPI_DBI_MAX_DATA_BUS_WIDTH 16

/* ESP-IDF inspired timing and transfer limits */
#define LCD_TRANS_QUEUE_DEPTH_DEFAULT 10
#define LCD_DMA_ALIGN_BYTES 4
#define LCD_PCLK_DIV_MAX 64
#define LCD_CLK_FRAC_DIV_N_MAX 256

struct mipi_dbi_esp_lcd_config
{
	/* Parallel 8080 data GPIOs */
	const struct gpio_dt_spec data[MIPI_DBI_MAX_DATA_BUS_WIDTH];
	const uint8_t bus_width;

	/* Read (type B) GPIO */
	const struct gpio_dt_spec rd;

	/* Write (type B) GPIO */
	const struct gpio_dt_spec wr;

	/* Chip-select GPIO */
	const struct gpio_dt_spec cs;

	/* Command/Data GPIO */
	const struct gpio_dt_spec cmd_data;

	/* Reset GPIO */
	const struct gpio_dt_spec reset;

	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	uint32_t lcd_clk;
	uint32_t max_transfer_bytes;

	/* DMA configuration */
	const struct device *dma_dev;
	uint32_t dma_channel;


};

struct mipi_dbi_esp_lcd_data
{
	/*
	 * LCD transaction synchronization. TRANS_DONE implies the GDMA read
	 * finished too (DMA always completes before the last byte is clocked
	 * out), so a single completion semaphore covers both.
	 */
	struct k_sem lcd_transaction_done;

	/* Serializes transactions between API callers */
	struct k_mutex lock;

	/* ESP-IDF interrupt handle */
	intr_handle_t intr_handle;
};

static void lcd_periph_trigger_quick_trans_done_event(lcd_cam_dev_t *lcd_dev)
{
    // trigger a quick interrupt event by a dummy transaction, wait the LCD interrupt line goes active
    // next time when esp_intr_enable is invoked, we can go into interrupt handler immediately
    // where we dispatch transactions for i80 devices
    lcd_ll_set_phase_cycles(lcd_dev, 0, 1, 0);
    lcd_ll_start(lcd_dev);
    // while (!(lcd_ll_get_interrupt_status(lcd_dev) & LCD_LL_EVENT_TRANS_DONE)) {}
}

static void IRAM_ATTR mipi_dbi_esp_lcd_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct mipi_dbi_esp_lcd_data *data = dev->data;
	lcd_cam_dev_t *lcd_dev = &LCD_CAM;
	uint32_t intr_status;

	intr_status = lcd_ll_get_interrupt_status(lcd_dev);

	if (intr_status & LCD_LL_EVENT_TRANS_DONE)
	{
		/* Clear the interrupt */
		lcd_ll_clear_interrupt_status(lcd_dev, LCD_LL_EVENT_TRANS_DONE);
		lcd_ll_stop(lcd_dev);

		/* Signal transaction completion */
		k_sem_give(&data->lcd_transaction_done);

		// LOG_DBG("LCD transaction completed");
	}
	else
	{
		LOG_WRN("LCD ISR called with unhandled status: 0x%x", intr_status);
		/* Clear all other interrupts */
		// lcd_ll_clear_interrupt_status(lcd_dev, intr_status);
		k_panic();
	}
}

static void mipi_dbi_esp_dma_callback(const struct device *dma_dev, void *user_data,
				      uint32_t channel, int status)
{
	struct mipi_dbi_esp_lcd_data *data = user_data;

	// LOG_DBG("DMA callback - channel %d, status %d", channel, status);

	ARG_UNUSED(data);

	/* DMA callback only checks status - LCD ISR signals completion */
	if (status != 0)
	{
		LOG_WRN("DMA transfer error: %d", status);
		k_panic();
	}
}

int mipi_dbi_esp_cmd_write(const struct device *dev, const struct mipi_dbi_config *config, uint8_t cmd, const uint8_t *data, size_t len)
{
	const struct mipi_dbi_esp_lcd_config *esp_config = dev->config;
	struct mipi_dbi_esp_lcd_data *esp_data = dev->data;
	lcd_cam_dev_t *lcd_dev = &LCD_CAM;
	int ret = 0;

	DMA_ATTR static uint8_t dma_buff[64];

	/* Validate that we only support 8080 parallel modes */
	if (config->mode != MIPI_DBI_MODE_8080_BUS_8_BIT &&
	    config->mode != MIPI_DBI_MODE_8080_BUS_16_BIT) {
		LOG_ERR("Unsupported MIPI mode: %d", config->mode);
		return -ENOTSUP;
	}

	/* Serialize callers; completion is awaited before returning below */
	k_mutex_lock(&esp_data->lock, K_FOREVER);
	k_sem_reset(&esp_data->lcd_transaction_done);

	// LOG_INF("cmd_write: cmd=0x%02x, len=%zu, mode=%d", cmd, len, config->mode);
	// if (len) { LOG_HEXDUMP_INF(data, len, "data:"); }

	// lcd_ll_reverse_bit_order(lcd_dev, false); // Ensure bit order is normal
	// lcd_ll_swap_byte_order(lcd_dev, config->mode, true);  // Enable byte swapping for big-endian ST7789V

	if (data && len && len < sizeof(dma_buff))
	{
		/* Configure GDMA for data transfer */
		struct dma_block_config dma_block =
		{
			.source_address = (uint32_t)dma_buff,
			.dest_address   = 0,
			.block_size     = len,
		};

		struct dma_config dma_cfg =
		{
			.channel_direction    = MEMORY_TO_PERIPHERAL,
			.dma_callback         = mipi_dbi_esp_dma_callback,
			.user_data            = esp_data,
			.dma_slot             = SOC_GDMA_TRIG_PERIPH_LCD0,
			.complete_callback_en = 1,
			.source_data_size     = len,
			.source_burst_length  = 0,
			.dest_data_size       = 0,
			.dest_burst_length    = 0,
			.head_block           = &dma_block
		};
		
		memcpy(dma_buff, data, len);

		ret = dma_config(esp_config->dma_dev, esp_config->dma_channel, &dma_cfg);
		if (ret != 0) {
			LOG_ERR("DMA config failed: %d", ret);
			k_mutex_unlock(&esp_data->lock);
			return ret;
		}
	}

	/*
	 * Reset TX FIFO before reconfiguring, like esp_lcd's i80 driver does
	 * for every transaction. Leftover FIFO bytes from the previous
	 * transaction otherwise leak into this one and the new phase
	 * configuration (command phase!) is not applied cleanly.
	 */
	lcd_ll_fifo_reset(lcd_dev);

	lcd_ll_set_command(lcd_dev, esp_config->bus_width, cmd);
	lcd_ll_set_dc_level(lcd_dev, 0, cmd ? 0 : 1, 0, data ? 1 : 0);
	lcd_ll_set_phase_cycles(lcd_dev, cmd ? 1 : 0, 0, (data && len > 0) ? len : 0);
	lcd_ll_enable_output_always_on(lcd_dev, true);
	// lcd_ll_set_blank_cycles(lcd_dev, 1, 1);  // Extra setup time for ST7789V commands - match display function timing

	// lcd_ll_set_dc_delay_ticks(lcd_dev, 1);

	//start dma transaction here
	if (data && len)
	{
		ret = dma_start(esp_config->dma_dev, esp_config->dma_channel);
		if (ret != 0) {
			LOG_ERR("DMA start failed: %d", ret);
			k_mutex_unlock(&esp_data->lock);
			return ret;
		}
		// k_busy_wait(5); //delay to ensure dma has transfered data to LCD FIFO
	}

	lcd_ll_start(lcd_dev);

	/*
	 * Block until the transfer fully completes so the caller may reuse
	 * or free its buffer immediately after return.
	 */
	k_sem_take(&esp_data->lcd_transaction_done, K_FOREVER);

	k_mutex_unlock(&esp_data->lock);

	return 0;
}

int mipi_dbi_esp_cmd_read(const struct device *dev, const struct mipi_dbi_config *config, uint8_t *cmds, size_t num_cmds, uint8_t *response, size_t len)
{
	return -ENOTSUP; // Not implemented for this driver
}

int mipi_dbi_esp_write_display(const struct device *dev, const struct mipi_dbi_config *config, const uint8_t *framebuf, struct display_buffer_descriptor *desc, enum display_pixel_format pixfmt)
{
	const struct mipi_dbi_esp_lcd_config *esp_config = dev->config;
	struct mipi_dbi_esp_lcd_data *esp_data = dev->data;
	lcd_cam_dev_t *lcd_dev = &LCD_CAM;
	int ret = 0;
	size_t buf_len;

	/* Calculate buffer length FIRST */
	buf_len = desc->buf_size;

	// LOG_DBG("write_display: buf_len=%zu bytes", buf_len);

	/* Serialize callers; completion is awaited before returning below */
	k_mutex_lock(&esp_data->lock, K_FOREVER);
	k_sem_reset(&esp_data->lcd_transaction_done);

	// lcd_ll_reverse_bit_order(lcd_dev, false); // Ensure bit order is normal
	// lcd_ll_swap_byte_order(lcd_dev, config->mode, true);  // Enable byte swapping for big-endian ST7789V

	/* Configure GDMA for data transfer */
	struct dma_block_config dma_block =
	{
		.source_address = (uint32_t)framebuf,
		.dest_address   = 0,
		.block_size     = buf_len,
	};

	struct dma_config dma_cfg =
	{
		.channel_direction    = MEMORY_TO_PERIPHERAL,
		.dma_callback         = mipi_dbi_esp_dma_callback,
		.user_data            = esp_data,
		.dma_slot             = SOC_GDMA_TRIG_PERIPH_LCD0,
		.complete_callback_en = 1,
		.source_data_size     = buf_len,
		.source_burst_length  = 0,
		.dest_data_size       = 0,
		.dest_burst_length    = 0,
		.head_block           = &dma_block
	};

	ret = dma_config(esp_config->dma_dev, esp_config->dma_channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("DMA config failed: %d", ret);
		goto cleanup;
	}

	/* Reset TX FIFO before reconfiguring (see comment in cmd_write) */
	lcd_ll_fifo_reset(lcd_dev);

	/* Set DC levels for data-only transfer: idle=0, cmd=1(data mode), dummy=0, data=1 */
	lcd_ll_set_dc_level(lcd_dev, 0, 1, 0, 1);
	/* Enable data phase only, use actual byte count for phase cycles */
	lcd_ll_set_phase_cycles(lcd_dev, 0, 0, buf_len);
	lcd_ll_enable_output_always_on(lcd_dev, true);
	lcd_ll_set_blank_cycles(lcd_dev, 1, 1);    // Standard blank cycles for bulk data

	/* Start DMA transfer */
	ret = dma_start(esp_config->dma_dev, esp_config->dma_channel);
	if (ret != 0) {
		LOG_ERR("DMA start failed: %d", ret);
		goto cleanup;
	}

	lcd_ll_start(lcd_dev);

	/*
	 * Block until DMA and the LCD transaction complete so the caller's
	 * framebuffer may be redrawn immediately after return. Returning
	 * while DMA still reads the buffer lets the application overwrite
	 * pixels in flight (stale-prefix tearing observed on logic analyzer).
	 */
	k_sem_take(&esp_data->lcd_transaction_done, K_FOREVER);

	ret = 0;

cleanup:
	k_mutex_unlock(&esp_data->lock);
	return ret;
}

int mipi_dbi_esp_reset(const struct device *dev, k_timeout_t delay)
{
	const struct mipi_dbi_esp_lcd_config *esp_config = dev->config;
	struct mipi_dbi_esp_lcd_data *esp_data = dev->data;

	if (gpio_is_ready_dt(&esp_config->reset))
	{
		/* Assert reset (active low) */
		gpio_pin_set_dt(&esp_config->reset, 1);

		/* Hold reset for specified delay */
		k_sleep(delay);

		/* Release reset */
		gpio_pin_set_dt(&esp_config->reset, 0);

		/* Wait for display to stabilize after reset */
		k_sleep(K_MSEC(10));
	}

	return 0;
}

int mipi_dbi_esp_release(const struct device *dev, const struct mipi_dbi_config *config)
{
	/* CS automatically goes inactive when LCD peripheral completes transaction */
	return -ENOTSUP;
}

int mipi_dbi_esp_configure_te(const struct device *dev, uint8_t edge, k_timeout_t delay)
{
	/* Tearing effect (TE) configuration not implemented for this driver.
	 * This would require additional GPIO configuration and interrupt handling
	 * for display synchronization.
	 */
	return -ENOTSUP;
}

static int mipi_dbi_esp_lcd_init(const struct device *dev)
{
	const struct mipi_dbi_esp_lcd_config *config = dev->config;
	struct mipi_dbi_esp_lcd_data *data = dev->data;
	const char *failed_pin = NULL;
	lcd_cam_dev_t *lcd_dev = &LCD_CAM;
	int ret = 0;

	/* Initialize LCD transaction synchronization primitives */
	k_sem_init(&data->lcd_transaction_done, 0, 1);
	k_mutex_init(&data->lock);

	/* Enable peripheral */
	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret) {
		LOG_ERR("Failed to enable clock: %d", ret);
		return ret;
	}

	if (!config->lcd_clk) {
		LOG_ERR("No lcd_clk specified");
		return -EINVAL;
	}

	/* Use 160MHz PLL divided down to the specified lcd_clk frequency */
	uint32_t div_integer = ESP32_CLK_CPU_PLL_160M / config->lcd_clk;
	/* Validate the desired frequency can be achieved with integer division */
	if (div_integer == 0 || div_integer > LCD_PCLK_DIV_MAX) {
		LOG_ERR("Invalid lcd_clk %u Hz. Divider %u out of range (1-%d)",
			config->lcd_clk, div_integer, LCD_PCLK_DIV_MAX);
		return -EINVAL;
	}

	LOG_DBG("LCD clock: PLL=160MHz, div=%u, target=%uHz", div_integer, config->lcd_clk);

	/* LCD peripheral doesn't require explicit HAL initialization */

	LOG_DBG("Configuring LCD peripheral for 8080 interface");

	/* Reset and initialize LCD peripheral - based on video driver pattern */
	lcd_ll_reset(lcd_dev);
	while (lcd_dev->lcd_user.lcd_reset == 1) { arch_nop(); };
	lcd_ll_fifo_reset(lcd_dev);
	while (lcd_dev->lcd_misc.lcd_afifo_reset == 1) { arch_nop(); };

	/* Enable clock and set source */
	lcd_ll_enable_clock(lcd_dev, true);
	lcd_ll_select_clk_src(lcd_dev, LCD_CLK_SRC_PLL160M);
	lcd_ll_set_group_clock_coeff(lcd_dev, div_integer, 0, 0);  /* Use calculated divider */

	/* Allocate and configure ESP-IDF interrupt */
	ret = esp_intr_alloc_intrstatus(ETS_LCD_CAM_INTR_SOURCE,
					// ESP_INTR_FLAG_INTRDISABLED |
					ESP_INTR_FLAG_IRAM |
					ESP_INTR_FLAG_LOWMED,
					(uint32_t)&lcd_dev->lc_dma_int_st,
					LCD_LL_EVENT_TRANS_DONE,
					mipi_dbi_esp_lcd_isr,
					(void *)dev,
					&data->intr_handle);
	if (ret != ESP_OK) {
		LOG_ERR("Failed to allocate LCD interrupt: %d", ret);
		return ret;
	}

	ret = esp_intr_enable(data->intr_handle);
	if (ret != ESP_OK) {
		LOG_ERR("Failed to enable LCD interrupt: %d", ret);
		esp_intr_free(data->intr_handle);
		return ret;
	}

	/* Disable LCD peripheral interrupt for ESP32-S3 */
	// lcd_ll_enable_interrupt(lcd_dev, LCD_LL_EVENT_TRANS_DONE, false);

	/* Clear LCD peripheral pending interrupt for ESP32-S3 */
	lcd_ll_clear_interrupt_status(lcd_dev, UINT32_MAX);

	/* Configure LCD peripheral for I80 mode - disable RGB mode */
	lcd_ll_enable_rgb_mode(lcd_dev, false);

	/* Disable yuv converter */
	lcd_ll_enable_color_convert(lcd_dev, false);

	/* Set data width for LCD peripheral */
	lcd_ll_set_data_wire_width(lcd_dev, config->bus_width);

	/* Enable LCD peripheral interrupt for ESP32-S3 */
	lcd_ll_enable_interrupt(lcd_dev, LCD_LL_EVENT_TRANS_DONE, true);

	/* Take semaphore before starting LCD transaction */
	// ret = k_sem_take(&data->lcd_transaction_done, K_FOREVER);
	// if (ret != 0)
	// {
	// 	LOG_ERR("Failed to acquire LCD semaphore for command transfer");
	// 	return -EBUSY;
	// }
	// trigger a quick "trans done" event, and wait for the interrupt line goes active
    // this could ensure we go into ISR handler next time we call `esp_intr_enable`
    // lcd_periph_trigger_quick_trans_done_event(lcd_dev);

	/* Configure IO Matrix for LCD peripheral signals */
	/* 8-bit parallel data bus - GPIOs 39,40,41,42,45,46,47,48 -> LCD_DATA[0:7] */
	for (int i = 0; i < config->bus_width; i++) {
		if (gpio_is_ready_dt(&config->data[i])) {
			// ret = gpio_pin_configure_dt(&config->data[i], GPIO_OUTPUT);
			// if (ret < 0) {
			// 	uint32_t abs_pin = gpio_dt2abs(&config->data[i]);
			// 	LOG_ERR("Failed to configure data GPIO[%d] (absolute GPIO%d): %d", i, abs_pin, ret);
			// 	failed_pin = "data";
			// 	goto fail;
			// }

			uint32_t abs_pin = gpio_dt2abs(&config->data[i]);
			LOG_INF("DT GPIO[%d]: controller=%s, DT_pin=%d -> absolute GPIO%d (configured successfully)",
					i, config->data[i].port->name, config->data[i].pin, abs_pin);

			LOG_DBG("Routing GPIO%d -> LCD_DATA_OUT%d (signal %d)",
				abs_pin, i, LCD_DATA_OUT0_IDX + i);
			// esp_rom_gpio_matrix_out(abs_pin, LCD_DATA_OUT0_IDX + i, false, false);
			// gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[abs_pin], PIN_FUNC_GPIO);
			gpio_ll_func_sel(NULL, abs_pin, PIN_FUNC_GPIO);
			esp_rom_gpio_connect_out_signal(abs_pin, LCD_DATA_OUT0_IDX + i, false, false);
		}
	}

	/* DC (Data/Command) signal - GPIO 7 -> LCD_DC */
	if (gpio_is_ready_dt(&config->cmd_data)) {
		// ret = gpio_pin_configure_dt(&config->cmd_data, GPIO_OUTPUT);
		// if (ret < 0) {
		// 	failed_pin = "cmd_data";
		// 	goto fail;
		// }
		uint32_t abs_pin = gpio_dt2abs(&config->cmd_data);
		LOG_DBG("Routing GPIO%d -> LCD_DC (signal %d)", abs_pin, LCD_DC_IDX);
		gpio_ll_func_sel(NULL, abs_pin, PIN_FUNC_GPIO);
		esp_rom_gpio_connect_out_signal(abs_pin, LCD_DC_IDX, false, false);
		// esp_rom_gpio_matrix_out(abs_pin, LCD_DC_IDX, false, false);
		// gpio_ll_output_enable(&GPIO, abs_pin);
		// gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[abs_pin], PIN_FUNC_GPIO);
	}

	/* WR (Write/Clock) signal - GPIO 8 -> LCD_PCLK */
	if (gpio_is_ready_dt(&config->wr)) {
		// ret = gpio_pin_configure_dt(&config->wr, GPIO_OUTPUT);
		// if (ret < 0) {
		// 	failed_pin = "wr";
		// 	goto fail;
		// }
		uint32_t abs_pin = gpio_dt2abs(&config->wr);
		LOG_DBG("Routing GPIO%d -> LCD_PCLK (signal %d)", abs_pin, LCD_PCLK_IDX);
		gpio_ll_func_sel(NULL, abs_pin, PIN_FUNC_GPIO);

		/* Invert WR signal for ST7789V compatibility - device tree configures as GPIO_ACTIVE_HIGH
		 * but ST7789V expects WR to be active LOW (data latched on rising edge of inverted signal) */
		bool invert_wr = (config->wr.dt_flags & GPIO_ACTIVE_LOW) == 0;  /* Invert if DT says ACTIVE_HIGH */
		esp_rom_gpio_connect_out_signal(abs_pin, LCD_PCLK_IDX, invert_wr, false);
		LOG_DBG("WR signal inversion: %s (DT flags: 0x%x)", invert_wr ? "enabled" : "disabled", config->wr.dt_flags);
	}

	//lcd_i80_switch_devices does this once on first tx to display
	lcd_ll_set_pixel_clock_prescale(lcd_dev, div_integer); //is 40 in esp idf

	/* Configure clock idle level based on WR signal polarity */
	bool wr_active_high = (config->wr.dt_flags & GPIO_ACTIVE_LOW) == 0;
	if (wr_active_high) {
		/* If WR is configured as ACTIVE_HIGH in DT but we're inverting it,
		 * set idle level to LOW so inverted signal idles HIGH (ST7789V expectation) */
		lcd_ll_set_clock_idle_level(lcd_dev, 0);
		LOG_DBG("Clock idle level set to LOW (inverted WR signal will idle HIGH)");
	} else {
		/* If WR is configured as ACTIVE_LOW in DT, set idle level HIGH */
		lcd_ll_set_clock_idle_level(lcd_dev, 1);
		LOG_DBG("Clock idle level set to HIGH (WR signal not inverted)");
	}

	lcd_ll_set_pixel_clock_edge(lcd_dev, true);  // Try rising edge for ST7789V
	lcd_ll_set_dc_level(lcd_dev, 1, 1, 1, 1);
	//cs gpio output disable

	/* CS (Chip Select) signal - GPIO 6 -> LCD_CS */
	if (gpio_is_ready_dt(&config->cs)) {
		// ret = gpio_pin_configure_dt(&config->cs, GPIO_OUTPUT_INACTIVE);
		// if (ret < 0) {
		// 	failed_pin = "cs";
		// 	goto fail;
		// }
		uint32_t abs_pin = gpio_dt2abs(&config->cs);
		LOG_DBG("Routing GPIO%d -> LCD_CS (signal %d)", abs_pin, LCD_CS_IDX);
		// esp_rom_gpio_matrix_out(abs_pin, LCD_CS_IDX, false, false);
		// gpio_ll_output_enable(&GPIO, abs_pin);
		// gpio_ll_iomux_func_sel(GPIO_PIN_MUX_REG[abs_pin], PIN_FUNC_GPIO);

		gpio_ll_func_sel(NULL, abs_pin, PIN_FUNC_GPIO);
		esp_rom_gpio_connect_out_signal(abs_pin, LCD_CS_IDX, false, false);
	}

	/* Reset pin - GPIO 5 (manual control, not routed through LCD peripheral) */
	if (gpio_is_ready_dt(&config->reset)) {
		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			failed_pin = "reset";
			goto fail;
		}
	}

	/* RD pin - GPIO 9 (not used in 8080 write-only mode, keep high) */
	if (gpio_is_ready_dt(&config->rd)) {
		ret = gpio_pin_configure_dt(&config->rd, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "rd";
			goto fail;
		}

		/* Log RD signal polarity configuration for debugging */
		LOG_DBG("RD signal configured as %s (DT flags: 0x%x)",
			(config->rd.dt_flags & GPIO_ACTIVE_LOW) ? "ACTIVE_LOW" : "ACTIVE_HIGH",
			config->rd.dt_flags);
	}

	/* Debug: Print actual GPIO numbers from device tree */
	// LOG_INF("=== DEVICE TREE GPIO VERIFICATION ===");
	// LOG_INF("Reset GPIO: %d", gpio_dt2abs(&config->reset));
	// LOG_INF("CS GPIO: %d", gpio_dt2abs(&config->cs));
	// LOG_INF("DC GPIO: %d", gpio_dt2abs(&config->cmd_data));
	// LOG_INF("WR GPIO: %d", gpio_dt2abs(&config->wr));
	// LOG_INF("RD GPIO: %d", gpio_dt2abs(&config->rd));
	// for (int i = 0; i < config->bus_width; i++) {
	// 	if (gpio_is_ready_dt(&config->data[i])) {
	// 		LOG_INF("DATA[%d] GPIO: %d", i, gpio_dt2abs(&config->data[i]));
	// 	}
	// }
	// LOG_INF("=== END DEVICE TREE GPIO VERIFICATION ===");

	return ret;

fail:
	LOG_ERR("Failed to configure %s GPIO pin.", failed_pin);
	return ret;
}

static DEVICE_API(mipi_dbi, mipi_dbi_esp_lcd_driver_api) =
{
	.command_write = mipi_dbi_esp_cmd_write,
	.command_read  = mipi_dbi_esp_cmd_read,
	.write_display = mipi_dbi_esp_write_display,
	.reset         = mipi_dbi_esp_reset,
	.release       = mipi_dbi_esp_release,
};

#define MIPI_DBI_ESP_LCD_INIT(n)                                                             \
	static const struct mipi_dbi_esp_lcd_config mipi_dbi_esp_lcd_config_##n =            \
	{                                                                                    \
		.data =                                                                          \
		{                                                                                \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 0,  {0}),  /* GPIO39 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 1,  {0}),  /* GPIO40 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 2,  {0}),  /* GPIO41 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 3,  {0}),  /* GPIO42 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 4,  {0}),  /* GPIO45 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 5,  {0}),  /* GPIO46 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 6,  {0}),  /* GPIO47 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 7,  {0}),  /* GPIO48 */       \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 8,  {0}),                     \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 9,  {0}),                     \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 10, {0}),                     \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 11, {0}),                     \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 12, {0}),                     \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 13, {0}),                     \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 14, {0}),                     \
			GPIO_DT_SPEC_INST_GET_BY_IDX_OR(n, data_gpios, 15, {0})                      \
		},                                                                               \
		.bus_width      = DT_INST_PROP(n, bus_width),                             \
		.rd             = GPIO_DT_SPEC_INST_GET(n, rd_gpios),                     \
		.wr             = GPIO_DT_SPEC_INST_GET(n, wr_gpios),                     \
		.cs             = GPIO_DT_SPEC_INST_GET(n, cs_gpios),                     \
		.cmd_data       = GPIO_DT_SPEC_INST_GET(n, dc_gpios),                     \
		.reset          = GPIO_DT_SPEC_INST_GET(n, reset_gpios),                  \
		.clock_dev      = DEVICE_DT_GET(DT_CLOCKS_CTLR_BY_IDX(DT_NODELABEL(lcd_cam), 0)), \
		.clock_subsys   =  (clock_control_subsys_t) DT_PHA_BY_IDX(DT_NODELABEL(lcd_cam), clocks, 0, offset), \
		.lcd_clk        = DT_INST_PROP(n, lcd_clock),                         \
		.dma_dev        = ESP32_DT_INST_DMA_CTLR(n, tx), \
		.dma_channel    = ESP32_DT_INST_DMA_CELL(n, tx, channel), \
	};                                                                                   \
	BUILD_ASSERT(DT_INST_PROP_LEN(n, data_gpios) <= MIPI_DBI_MAX_DATA_BUS_WIDTH,         \
		         "Number of data GPIOs in DT exceeds MIPI_DBI_MAX_DATA_BUS_WIDTH");      \
	static struct mipi_dbi_esp_lcd_data mipi_dbi_esp_lcd_data_##n;                       \
	DEVICE_DT_INST_DEFINE(n, mipi_dbi_esp_lcd_init, NULL, &mipi_dbi_esp_lcd_data_##n,    \
                          &mipi_dbi_esp_lcd_config_##n, POST_KERNEL,                     \
                          CONFIG_MIPI_DBI_INIT_PRIORITY, &mipi_dbi_esp_lcd_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MIPI_DBI_ESP_LCD_INIT)

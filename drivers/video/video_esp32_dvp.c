/*
 * Copyright (c) 2024 espros photonics Co.
 * Copyright (c) 2024 Espressif Systems (Shanghai) CO LTD.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_lcd_cam

#include <soc/gdma_channel.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/esp32_clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <zephyr/kernel.h>
#include <hal/cam_hal.h>
#include <hal/cam_ll.h>

#include <hal/gdma_hal.h>
#include <hal/gdma_ll.h>
#include <soc/gdma_channel.h>
#include <hal/dma_types.h>

#include <zephyr/sys/crc.h>

#include <zephyr/logging/log.h>

#include "video_device.h"

LOG_MODULE_REGISTER(video_esp32_lcd_cam, CONFIG_VIDEO_LOG_LEVEL);

#define VIDEO_ESP32_DMA_BUFFER_MAX_SIZE 4092
#define VIDEO_ESP32_VSYNC_MASK          0x04


#ifdef CONFIG_POLL
#define VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(result)                                               \
	if (data->signal_out) {                                                                    \
		k_poll_signal_raise(data->signal_out, result);                                     \
	}
#else
#define VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(result)
#endif

enum video_esp32_cam_clk_sel_values {
	VIDEO_ESP32_CAM_CLK_SEL_NONE = 0,
	VIDEO_ESP32_CAM_CLK_SEL_XTAL = 1,
	VIDEO_ESP32_CAM_CLK_SEL_PLL_DIV2 = 2,
	VIDEO_ESP32_CAM_CLK_SEL_PLL_F160M = 3,
};

struct video_esp32_config {
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock_dev;
	const clock_control_subsys_t clock_subsys;
	const struct device *dma_dev;
	const struct device *source_dev;
	uint32_t cam_clk;
	uint8_t rx_dma_channel;
	uint8_t data_width;
	uint8_t invert_de;
	uint8_t invert_byte_order;
	uint8_t invert_bit_order;
	uint8_t invert_pclk;
	uint8_t invert_hsync;
	uint8_t invert_vsync;
	int irq_source;
	int irq_priority;
	int irq_flags;
};

struct video_esp32_data {
	cam_hal_context_t hal;
	const struct video_esp32_config *config;
	struct video_format video_format;
	struct video_buffer *active_vbuf;
	bool is_streaming;
	struct k_fifo fifo_in;
	struct k_fifo fifo_out;
	struct video_buffer *reload_on_vsync;
	struct dma_block_config dma_blocks[CONFIG_DMA_ESP32_MAX_DESCRIPTOR_NUM];
#ifdef CONFIG_POLL
	struct k_poll_signal *signal_out;
#endif
};

static int video_esp32_reload_dma(struct video_esp32_data *data);
static int video_esp32_set_stream(const struct device *dev, bool enable, enum video_buf_type type);


static void print_register_info(const volatile uint32_t* value, const char* label) {
	char buffer[36]; // 32 bits + 4 spaces + null terminator
	int index = 0;
	uint32_t i = 32;

    do {
		i--;
		if ((i+1) % 8 == 0 && i != 0) {
			buffer[index] = ' '; // Optional: space every byte
			index++;
		}
		buffer[index] = (*value & (1U << i)) ? '1' : '0';
		index++;		
	} while (i != 0 );
	buffer[index] = '\0';
    LOG_INF("val = 0b%s, addr = %p - Label: %s", buffer, value, label);
}


static void print_camera_registers(cam_hal_context_t *hal) {
	
	lcd_cam_lc_dma_int_ena_reg_t *lcd_cam_lc_dma_int_ena_reg = (lcd_cam_lc_dma_int_ena_reg_t *)&(hal->hw->lc_dma_int_ena.val);
	lcd_cam_cam_ctrl_reg_t *cam_ctrl = (lcd_cam_cam_ctrl_reg_t *)&(hal->hw->cam_ctrl.val);
	lcd_cam_cam_ctrl1_reg_t *cam_ctrl1 = (lcd_cam_cam_ctrl1_reg_t *)&(hal->hw->cam_ctrl1.val);
	
	LOG_ERR("Camera Registers at device %p", hal->hw);
	print_register_info(&(lcd_cam_lc_dma_int_ena_reg->val), "lcd_cam_lc_dma_int_ena_reg");
	print_register_info(&(cam_ctrl->val), "lcd_cam_cam_ctrl_reg");
	print_register_info(&(cam_ctrl1->val), "lcd_cam_cam_ctrl1_reg");
}

static void print_gdma_registers(gdma_dev_t *gdma_dev, uint8_t channel) {
	LOG_ERR("GDMA Registers for channel %d, at device %p", channel, gdma_dev);
	print_register_info(&(gdma_dev->channel[channel].in.int_ena.val), "int_ena");
	print_register_info(&(gdma_dev->channel[channel].in.conf0.val), "int_st");
	print_register_info(&(gdma_dev->channel[channel].in.link.val), "in_link");
}



static int get_jpeg_size(struct video_buffer *vbuf)
{
	uint8_t *buffer = (uint8_t *)vbuf->buffer;
	size_t max_size = vbuf->size;
	size_t jpeg_start = (size_t) - 1;
	size_t jpeg_end = (size_t) - 1;
	size_t non_zero_count = 0;
	
	/* Count non-zero bytes */
	for (size_t i = 0; i < max_size; i++) {
		if (buffer[i] != 0xFF) {
			non_zero_count++;
		}
	}
	LOG_INF("Non-zero bytes in buffer: %zu / %zu", non_zero_count, max_size);
	
	/* Find JPEG Start of Image marker (0xFFD8) */
	for (size_t i = 0; i < max_size - 1; i++) {
		if (buffer[i] == 0xFF && buffer[i + 1] == 0xD8) {
			jpeg_start = i;
			LOG_INF("Found SOI at offset %zu", i);
			break;
		}
	}

	if (jpeg_start == (size_t)-1) {
		LOG_ERR("JPEG SOI marker (0xFFD8) not found");
		jpeg_start = 0;  /* Use full buffer */
	}
	
	/* Check for critical JPEG markers */
	bool found_sos = false;
	bool found_sof = false;
	bool found_dqt = false;
	bool found_dht = false;
	size_t sos_offset = 0;
	
	for (size_t i = jpeg_start; i < max_size - 1; i++) {
		if (buffer[i] == 0xFF) {
			uint8_t marker = buffer[i + 1];
			switch (marker) {
			case 0xDA: /* SOS - Start of Scan */
				found_sos = true;
				sos_offset = i;
				LOG_INF("Found SOS at offset %zu", i);
				break;
			case 0xC0: /* SOF0 - Start of Frame (Baseline DCT) */
			case 0xC2: /* SOF2 - Start of Frame (Progressive DCT) */
				found_sof = true;
				LOG_INF("Found SOF at offset %zu", i);
				break;
			case 0xDB: /* DQT - Define Quantization Table */
				found_dqt = true;
				LOG_INF("Found DQT at offset %zu", i);
				break;
			case 0xC4: /* DHT - Define Huffman Table */
				found_dht = true;
				LOG_INF("Found DHT at offset %zu", i);
				break;
			}
		}
	}
	
	LOG_INF("JPEG markers: SOI=%d SOF=%d DQT=%d DHT=%d SOS=%d",
		jpeg_start != (size_t)-1, found_sof, found_dqt, found_dht, found_sos);
	
	if (!found_sos) {
		LOG_ERR("CRITICAL: SOS (Start of Scan) marker (0xFFDA) missing - JPEG incomplete!");
	}
	
	/* Scan for JPEG End of Image marker (0xFFD9) starting from SOI */
	for (size_t i = jpeg_start; i < max_size - 1; i++) {
		if (buffer[i] == 0xFF && buffer[i + 1] == 0xD9) {
			/* Found EOI marker, include the 2-byte marker */
			jpeg_end = i + 2;
			LOG_INF("Found EOI at offset %zu", i);
			break;
		}
	}
	
	/* If no EOI found, use full buffer */
	if (jpeg_end == (size_t)-1) {
		LOG_ERR("JPEG EOI marker (0xFFD9) NOT FOUND - Image truncated!");
		LOG_ERR("Buffer filled completely (%zu bytes) - INCREASE BUFFER SIZE!", max_size);
		jpeg_end = max_size;
	} else {
		size_t unused_buffer = max_size - jpeg_end;
		if (unused_buffer < (max_size / 10)) {
			LOG_WRN("Buffer almost full! Only %zu bytes unused. Consider increasing buffer size.",
				unused_buffer);
		} else {
			LOG_INF("Buffer utilization: %zu / %zu bytes (%.1f%%), %zu bytes free",
				jpeg_end, max_size, (float)jpeg_end * 100.0f / max_size, unused_buffer);
		}
		
		/* Check compressed image data size */
		if (found_sos && sos_offset > 0) {
			size_t compressed_data_size = (jpeg_end - 2) - (sos_offset + 2);
			size_t header_size = sos_offset - jpeg_start;
			LOG_INF("JPEG structure: Header=%zu bytes, Compressed data=%zu bytes",
				header_size, compressed_data_size);
			
			/* Warn if compressed data seems too small */
			if (compressed_data_size < 5000) {
				LOG_ERR("Compressed data suspiciously small (%zu bytes) - frame may be cut short!",
					compressed_data_size);
				LOG_ERR("This suggests DMA/camera stopped early despite EOI being present");
			}
		}
	}
	
	/* If JPEG doesn't start at beginning, move it */
	if (jpeg_start > 0) {
		size_t jpeg_size = jpeg_end - jpeg_start;
		LOG_ERR("JPEG Move disabled for debugging purposes.");
		LOG_ERR("Moving JPEG from offset %zu to beginning (%zu bytes)", jpeg_start, jpeg_size);
		//memmove(buffer, buffer + jpeg_start, jpeg_size);
		return jpeg_size;
	}
	
    /* Calculate CRC32 of the JPEG data */
    // size_t jpeg_size = jpeg_end - jpeg_start;
    // uint32_t crc = crc32_ieee(buffer + jpeg_start, jpeg_size);
    // LOG_INF("JPEG CRC32: 0x%08X (size: %zu bytes)", crc, jpeg_size);
   

	return jpeg_end - jpeg_start;
}
static int dma_esp32_get_status(gdma_dev_t *dev, uint32_t channel)
{
	dma_descriptor_t *desc;
	struct dma_status status;
	status.busy = !gdma_ll_rx_is_fsm_idle(dev, channel);
	status.dir = PERIPHERAL_TO_MEMORY;
	desc = (dma_descriptor_t *)gdma_ll_rx_get_current_desc_addr(
		dev, channel);
	
	// if (desc >= dma_channel->desc_list) {
	// 	status.read_position = desc - dma_channel->desc_list;
	// 	status.total_copied = desc->dw0.length
	// 				+ dma_channel->desc_list[0].dw0.size
	// 				* status.read_position;
	// }
	if (status.busy) {
		LOG_ERR("DMA is busy");
	} else {
		LOG_ERR("DMA is idle");
	}

	LOG_ERR("Descriptor %p, buffer pointer %p size %d, length: %d", desc, desc->buffer, desc->dw0.size, desc->dw0.length);
	return 0;
}

static void IRAM_ATTR video_esp32_vsync_isr(const struct device *dev)
{
	struct video_esp32_data *data = dev->data;
	uint32_t status = data->hal.hw->lc_dma_int_st.val;

	/* Check for VSYNC interrupt */
	if (status & VIDEO_ESP32_VSYNC_MASK) {
		/* Clear VSYNC interrupt */
		data->hal.hw->lc_dma_int_clr.val = VIDEO_ESP32_VSYNC_MASK;
		

		LOG_WRN("VSYNC interrupt triggered <--");
		/* Reload DMA for next frame on vsync */
		// if (data->reload_on_vsync) {
		// 	LOG_WRN("Reloading DMA on VSYNC");
		// 	data->active_vbuf = data->reload_on_vsync;
		// 	video_esp32_reload_dma(data);
		// 	data->reload_on_vsync = NULL;
		// }
		video_esp32_reload_dma(data);
		//dma_esp32_get_status(0x6003f000, 1);



		get_jpeg_size(data->active_vbuf);
		// memset(data->active_vbuf->buffer, 0, data->active_vbuf->size);		
	}
}

static int video_esp32_reload_dma(struct video_esp32_data *data)
{
	const struct video_esp32_config *cfg = data->config;
	int ret = 0;

	// if (data->active_vbuf == NULL) {
	// 	LOG_ERR("No video buffer available. Enqueue some buffers first.");
	// 	return -EAGAIN;
	// }

	ret = dma_reload(cfg->dma_dev, cfg->rx_dma_channel, 0, (uint32_t)data->active_vbuf->buffer,
			 data->active_vbuf->size);
	if (ret) {
		LOG_ERR("Unable to reload DMA (%d)", ret);
		return ret;
	}

	// ret = dma_start(cfg->dma_dev, cfg->rx_dma_channel);
	// if (ret) {
	// 	LOG_ERR("Unable to start DMA (%d)", ret);
	// 	return ret;
	// }

	return 0;
}

void video_esp32_dma_rx_done(const struct device *dev, void *user_data, uint32_t channel,
			     int status)
{
	struct video_esp32_data *data = user_data;

	//LOG_ERR("DMA RX done with status: %d", status);
	//LOG_ERR("Should never occur because of circular DMA descriptors");

	//get_jpeg_size(data->active_vbuf);
	//memset(data->active_vbuf->buffer, 0, data->active_vbuf->size);	
	//return;

	if (status == DMA_STATUS_BLOCK) {
		LOG_WRN("received block");
		return;
	}

	if (status != DMA_STATUS_COMPLETE) {
		VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_ERROR)
		LOG_ERR("DMA error: %d", status);
		return;
	}

	if (data->active_vbuf == NULL) {
		VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_ERROR)
		LOG_ERR("No video buffer available. Enque some buffers first.");
		return;
	}
	LOG_WRN("Reloading DMA for next frame");
	const struct video_esp32_config *cfg = data->config;
	int ret = dma_reload(cfg->dma_dev, cfg->rx_dma_channel, 0, (uint32_t)data->active_vbuf->buffer,
			 data->active_vbuf->size);

	if (ret == 3) {
		k_fifo_put(&data->fifo_out, data->active_vbuf);
		VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_DONE)
		data->active_vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);

		if (data->active_vbuf == NULL) {
			LOG_WRN("Frame dropped. No buffer available");
			//VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_ERROR)
			//video_esp32_set_stream(dev, false, VIDEO_BUF_TYPE_OUTPUT);
		}		
		return;
	}

	if (ret) {
		LOG_ERR("Unable to reload DMA (%d)", ret);
		get_jpeg_size(data->active_vbuf);
		return;
	}	

	LOG_INF("GDMA ISR");
	cam_hal_context_t* hal = &data->hal;
	print_camera_registers(hal);	


	// cam_ll_stop(hal->hw);
    // cam_ll_reset(hal->hw);
    // cam_ll_fifo_reset(hal->hw);
    // cam_ll_start(hal->hw);
	// lcd_cam_dev_t *dev_cam = (lcd_cam_dev_t *)hal->hw;
	// dev_cam->cam_ctrl.cam_update = 1;

	//get_jpeg_size(data->active_vbuf);
	LOG_WRN("End of GDMA ISR <--");

	if( video_esp32_reload_dma(data) == 3 ) {
		k_fifo_put(&data->fifo_out, data->active_vbuf);
		VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_DONE)
		data->active_vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);

		if (data->active_vbuf == NULL) {
			LOG_WRN("Frame dropped. No buffer available");
			//VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_ERROR)
			//video_esp32_set_stream(dev, false, VIDEO_BUF_TYPE_OUTPUT);
			return;
		}		
	}
	// k_fifo_put(&data->fifo_out, data->active_vbuf);
	// VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_DONE)
	// data->active_vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);

	// if (data->active_vbuf == NULL) {
	// 	LOG_WRN("Frame dropped. No buffer available");
	// 	//VIDEO_ESP32_RAISE_OUT_SIG_IF_ENABLED(VIDEO_BUF_ERROR)
	// 	//video_esp32_set_stream(dev, false, VIDEO_BUF_TYPE_OUTPUT);
	// 	return;
	// }
	

	
	// data->reload_on_vsync = data->active_vbuf;
}



static int video_esp32_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	const struct video_esp32_config *cfg = dev->config;
	struct video_esp32_data *data = dev->data;
	struct dma_status dma_status = {0};
	struct dma_config dma_cfg = {0};
	struct dma_block_config *dma_block_iter = data->dma_blocks;
	uint32_t buffer_size = 0;
	int error = 0;

	if (!enable) {
		LOG_WRN("Stop streaming");

		cam_hal_stop_streaming(&data->hal);
		
		error = dma_stop(cfg->dma_dev, cfg->rx_dma_channel);
		if (error) {
			LOG_ERR("Unable to stop DMA (%d)", error);
			return error;
		}

		if (video_stream_stop(cfg->source_dev, type)) {
			return -EIO;
		}		

		

		data->is_streaming = false;
		return 0;
	}

	if (data->is_streaming) {
		return -EBUSY;
	}

	LOG_DBG("Start streaming");

	error = dma_get_status(cfg->dma_dev, cfg->rx_dma_channel, &dma_status);

	if (error) {
		LOG_ERR("Unable to get Rx status (%d)", error);
		return error;
	}

	if (dma_status.busy) {
		LOG_ERR("Rx DMA Channel %d is busy", cfg->rx_dma_channel);
		return -EBUSY;
	}

	data->active_vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT);
	if (!data->active_vbuf) {
		LOG_ERR("No enqueued video buffers available.");
		return -EAGAIN;
	}

	buffer_size = data->active_vbuf->size;
	memset(data->dma_blocks, 0, sizeof(data->dma_blocks));
	for (int i = 0; i < CONFIG_DMA_ESP32_MAX_DESCRIPTOR_NUM; ++i) {
		dma_block_iter->dest_address =
			(uint32_t)data->active_vbuf->buffer + (i * VIDEO_ESP32_DMA_BUFFER_MAX_SIZE);
		if (buffer_size < VIDEO_ESP32_DMA_BUFFER_MAX_SIZE) {
			dma_block_iter->block_size = buffer_size;
			dma_block_iter->next_block = NULL;
			dma_cfg.block_count = i + 1;
			break;
		}
		dma_block_iter->block_size = VIDEO_ESP32_DMA_BUFFER_MAX_SIZE;
		dma_block_iter->next_block = dma_block_iter + 1;
		dma_block_iter++;
		buffer_size -= VIDEO_ESP32_DMA_BUFFER_MAX_SIZE;
	}

	if (dma_block_iter->next_block) {
		LOG_ERR("Not enough descriptors available. Increase "
			"CONFIG_DMA_ESP32_MAX_DESCRIPTOR_NUM");
		return -ENOBUFS;
	}

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.dma_callback = video_esp32_dma_rx_done;
	dma_cfg.user_data = data;
	dma_cfg.dma_slot = SOC_GDMA_TRIG_PERIPH_CAM0;
	dma_cfg.complete_callback_en = 1;
	dma_cfg.head_block = &data->dma_blocks[0];

	LOG_INF("Start the camera...");
	if (video_stream_start(cfg->source_dev, type)) {
		return -EIO;
	}	

	LOG_INF("Configure DMA...");
	error = dma_config(cfg->dma_dev, cfg->rx_dma_channel, &dma_cfg);
	if (error) {
		LOG_ERR("Unable to configure DMA (%d)", error);
		return error;
	}

	

	LOG_INF("Starting DMA...");
	error = dma_start(cfg->dma_dev, cfg->rx_dma_channel);
	if (error) {
		LOG_ERR("Unable to start DMA (%d)", error);
		return error;
	}
	
	cam_hal_context_t* hal = &data->hal;
	print_camera_registers(hal);	


	print_gdma_registers(0x6003f000, 1);
	LOG_WRN("Alexanders hacking starts here");
	// lcd_cam_lc_dma_int_ena_reg_t *lcd_cam_lc_dma_int_ena_reg = (lcd_cam_lc_dma_int_ena_reg_t *)&(hal->hw->lc_dma_int_ena.val);
	// lcd_cam_cam_ctrl_reg_t *cam_ctrl = (lcd_cam_cam_ctrl_reg_t *)&(hal->hw->cam_ctrl.val);
	// lcd_cam_cam_ctrl1_reg_t *cam_ctrl1 = (lcd_cam_cam_ctrl1_reg_t *)&(hal->hw->cam_ctrl1.val);
	// lcd_cam_lc_dma_int_ena_reg->lcd_trans_done_int_ena.val = 0;

	lcd_cam_dev_t *dev_cam = (lcd_cam_dev_t *)hal->hw;
	//dev_cam->lc_dma_int_ena.val = 0;
	dev_cam->lc_dma_int_ena.cam_vsync_int_ena = 0;
	//dev_cam->cam_ctrl.cam_vsync_filter_thres = 0x0003; // Reduce VSYNC filter threshold to 3 cycles
	
	dev_cam->cam_ctrl.cam_vs_eof_en = 0;
	dev_cam->cam_ctrl1.cam_rec_data_bytelen = 40000; // Hack to force read of byte length

	dev_cam->cam_ctrl.cam_update = 1;

	//k_sleep(K_MSEC(1000));
	// printk("Camera interrupt control register: \n");
	// print_binary(lcd_cam_lc_dma_int_ena_reg->val);
	
	// lcd_cam_lc_dma_int_ena_reg->cam_vsync_int_ena = 0;
	// lcd_cam_lc_dma_int_ena_reg->cam_hs_int_ena = 0;

	// lcd_cam_cam_ctrl_reg_t *cam_ctrl = (lcd_cam_cam_ctrl_reg_t *)&(data->hal.hw->cam_ctrl.val);
	// cam_ctrl->cam_line_int_en = 0;
	
	// lcd_cam_cam_ctrl1_reg_t *cam_ctrl1 = (lcd_cam_cam_ctrl1_reg_t *)&(data->hal.hw->cam_ctrl1.val);
	// LOG_INF("Reading %d bytes", cam_ctrl1->cam_rec_data_bytelen);
	// cam_ctrl1->cam_rec_data_bytelen = 6000; // Hack to force read of byte length
	// LOG_WRN("Alexanders hacking ends here");

	LOG_WRN("Registers while initializing:");
	print_camera_registers(&data->hal);	

	cam_hal_start_streaming(&data->hal);
	dev_cam->cam_ctrl.cam_update = 1;	





	data->is_streaming = true;

	LOG_WRN("Registers while running:");
	print_camera_registers(&data->hal);

	return 0;
}

static int video_esp32_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct video_esp32_config *config = dev->config;

	/* ESP32 produces full frames */
	caps->min_line_count = caps->max_line_count = LINE_COUNT_HEIGHT;

	/* Forward the message to the source device */
	return video_get_caps(config->source_dev, caps);
}

static int video_esp32_get_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_esp32_config *cfg = dev->config;
	int ret = 0;

	LOG_DBG("Get format");

	ret = video_get_format(cfg->source_dev, fmt);
	if (ret) {
		LOG_ERR("Failed to get format from source");
		return ret;
	}

	fmt->pitch = fmt->width * video_bits_per_pixel(fmt->pixelformat) / BITS_PER_BYTE;

	return 0;
}

static int video_esp32_set_fmt(const struct device *dev, struct video_format *fmt)
{
	const struct video_esp32_config *cfg = dev->config;
	struct video_esp32_data *data = dev->data;
	int ret;

	ret = video_set_format(cfg->source_dev, fmt);
	if (ret < 0) {
		return ret;
	}

	fmt->pitch = fmt->width * video_bits_per_pixel(fmt->pixelformat) / BITS_PER_BYTE;

	data->video_format = *fmt;

	/* Query and log camera sensor settings */
	struct video_control ctrl;
	int ctrl_ret;
	
	LOG_INF("=== Camera Sensor Configuration ===");
	
	/* Check JPEG compression quality */
	ctrl.id = VIDEO_CID_JPEG_COMPRESSION_QUALITY;
	ctrl_ret = video_get_ctrl(cfg->source_dev, &ctrl);
	if (ctrl_ret == 0) {
		LOG_INF("JPEG Compression Quality: %d (0-63, higher=better)", ctrl.val);
	} else {
		LOG_WRN("JPEG Compression Quality: Not supported (ret=%d)", ctrl_ret);
	}
	
	/* Check horizontal flip */
	ctrl.id = VIDEO_CID_HFLIP;
	ctrl_ret = video_get_ctrl(cfg->source_dev, &ctrl);
	if (ctrl_ret == 0) {
		LOG_INF("Horizontal Flip: %s", ctrl.val ? "ENABLED (may cause cropping)" : "disabled");
	} else {
		LOG_DBG("Horizontal Flip: Not readable (ret=%d)", ctrl_ret);
	}
	
	/* Check vertical flip */
	ctrl.id = VIDEO_CID_VFLIP;
	ctrl_ret = video_get_ctrl(cfg->source_dev, &ctrl);
	if (ctrl_ret == 0) {
		LOG_INF("Vertical Flip: %s", ctrl.val ? "ENABLED (may cause cropping)" : "disabled");
	} else {
		LOG_DBG("Vertical Flip: Not readable (ret=%d)", ctrl_ret);
	}
	
	/* Check test pattern */
	ctrl.id = VIDEO_CID_TEST_PATTERN;
	ctrl_ret = video_get_ctrl(cfg->source_dev, &ctrl);
	if (ctrl_ret == 0) {
		LOG_INF("Test Pattern: %s", ctrl.val ? "ENABLED" : "disabled");
	} else {
		LOG_DBG("Test Pattern: Not readable (ret=%d)", ctrl_ret);
	}
	
	/* Check brightness */
	ctrl.id = VIDEO_CID_BRIGHTNESS;
	ctrl_ret = video_get_ctrl(cfg->source_dev, &ctrl);
	if (ctrl_ret == 0) {
		LOG_INF("Brightness: %d", ctrl.val);
	}
	
	/* Check contrast */
	ctrl.id = VIDEO_CID_CONTRAST;
	ctrl_ret = video_get_ctrl(cfg->source_dev, &ctrl);
	if (ctrl_ret == 0) {
		LOG_INF("Contrast: %d", ctrl.val);
	}
	
	/* Check saturation */
	ctrl.id = VIDEO_CID_SATURATION;
	ctrl_ret = video_get_ctrl(cfg->source_dev, &ctrl);
	if (ctrl_ret == 0) {
		LOG_INF("Saturation: %d", ctrl.val);
	}
	
	LOG_INF("=== End Camera Configuration ===");

	return 0;
}

static int video_esp32_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct video_esp32_data *data = dev->data;

	vbuf->bytesused = data->video_format.pitch * data->video_format.height;
	vbuf->line_offset = 0;

	k_fifo_put(&data->fifo_in, vbuf);

	if (data->active_vbuf == NULL && data->is_streaming) {
		LOG_WRN("Attempting to restart DMA after enqueue");
		data->reload_on_vsync = vbuf;
		// data->active_vbuf = vbuf;
		// video_esp32_reload_dma(data);
	}

	return 0;
}

static int video_esp32_dequeue(const struct device *dev, struct video_buffer **vbuf,
			       k_timeout_t timeout)
{
	struct video_esp32_data *data = dev->data;

	*vbuf = k_fifo_get(&data->fifo_out, timeout);

	/* For JPEG format, calculate actual size from buffer */
	if (data->video_format.pixelformat == VIDEO_PIX_FMT_JPEG) {
		size_t jpeg_size = get_jpeg_size((*vbuf));
		(*vbuf)->bytesused = jpeg_size;
	}

	LOG_WRN("Dequeue done, vbuf = %p, bytesused %zu", *vbuf, (*vbuf)->bytesused);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}
	return 0;
}

static int video_esp32_flush(const struct device *dev, bool cancel)
{
	struct video_esp32_data *data = dev->data;
	struct video_buffer *vbuf = NULL;

	if (cancel) {
		if (data->active_vbuf) {
			k_fifo_put(&data->fifo_out, data->active_vbuf);
			data->active_vbuf = NULL;
		}
		while ((vbuf = k_fifo_get(&data->fifo_in, K_NO_WAIT)) != NULL) {
			k_fifo_put(&data->fifo_out, vbuf);
#ifdef CONFIG_POLL
			if (data->signal_out) {
				k_poll_signal_raise(data->signal_out, VIDEO_BUF_ABORTED);
			}
#endif
		}
	} else {
		while (!k_fifo_is_empty(&data->fifo_in)) {
			k_sleep(K_MSEC(1));
		}
	}

	return 0;
}

#ifdef CONFIG_POLL
int video_esp32_set_signal(const struct device *dev, struct k_poll_signal *sig)
{
	struct video_esp32_data *data = dev->data;

	data->signal_out = sig;
	return 0;
}
#endif

static void video_esp32_cam_ctrl_init(const struct device *dev)
{
	const struct video_esp32_config *cfg = dev->config;
	struct video_esp32_data *data = dev->data;

	const cam_hal_config_t hal_cfg = {
		.port = 0,
		.byte_swap_en = cfg->invert_byte_order,
	};

	cam_hal_init(&data->hal, &hal_cfg);

	cam_ll_reverse_dma_data_bit_order(data->hal.hw, cfg->invert_bit_order);
	cam_ll_enable_invert_pclk(data->hal.hw, cfg->invert_pclk);
	cam_ll_set_input_data_width(data->hal.hw, cfg->data_width);
	cam_ll_enable_invert_de(data->hal.hw, cfg->invert_de);
	cam_ll_enable_invert_vsync(data->hal.hw, cfg->invert_vsync);
	cam_ll_enable_invert_hsync(data->hal.hw, cfg->invert_hsync);

	LOG_ERR("Alex HAL CAM settings applied");
	cam_ll_enable_vsync_filter(data->hal.hw, true);

	//cam_ll_enable_stop_signal(data->hal.hw, false);


}

static int video_esp32_init(const struct device *dev)
{
	const struct video_esp32_config *cfg = dev->config;
	struct video_esp32_data *data = dev->data;

	k_fifo_init(&data->fifo_in);
	k_fifo_init(&data->fifo_out);
	data->config = cfg;
	video_esp32_cam_ctrl_init(dev);

	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	/* Configure VSYNC interrupt */
	int ret = esp_intr_alloc(cfg->irq_source,
				 ESP_PRIO_TO_FLAGS(cfg->irq_priority) | 
				 ESP_INT_FLAGS_CHECK(cfg->irq_flags) | ESP_INTR_FLAG_IRAM,
				 (intr_handler_t)video_esp32_vsync_isr,
				 (void *)dev,
				 NULL);
	if (ret != 0) {
		LOG_ERR("Could not allocate vsync interrupt handler (%d)", ret);
		return ret;
	}

	// /* Enable VSYNC interrupt in hardware */
	data->hal.hw->lc_dma_int_ena.val |= VIDEO_ESP32_VSYNC_MASK;

	return 0;
}

static DEVICE_API(video, esp32_driver_api) = {
	/* mandatory callbacks */
	.set_format = video_esp32_set_fmt,
	.get_format = video_esp32_get_fmt,
	.set_stream = video_esp32_set_stream,
	.get_caps = video_esp32_get_caps,
	/* optional callbacks */
	.enqueue = video_esp32_enqueue,
	.dequeue = video_esp32_dequeue,
	.flush = video_esp32_flush,
#ifdef CONFIG_POLL
	.set_signal = video_esp32_set_signal,
#endif
};

PINCTRL_DT_INST_DEFINE(0);

#define SOURCE_DEV(n) DEVICE_DT_GET(DT_INST_PHANDLE(n, source))

static const struct video_esp32_config esp32_config = {
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
	.source_dev = SOURCE_DEV(0),
	.dma_dev = ESP32_DT_INST_DMA_CTLR(0, rx),
	.rx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(0, rx, channel),
	.data_width = DT_INST_PROP_OR(0, data_width, 8),
	.invert_bit_order = DT_INST_PROP(0, invert_bit_order),
	.invert_byte_order = DT_INST_PROP(0, invert_byte_order),
	.invert_pclk = DT_INST_PROP(0, invert_pclk),
	.invert_de = DT_INST_PROP(0, invert_de),
	.invert_hsync = DT_INST_PROP(0, invert_hsync),
	.invert_vsync = DT_INST_PROP(0, invert_vsync),
	.cam_clk = DT_INST_PROP_OR(0, cam_clk, 0),
	.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(0)),
	.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(0, offset),
	.irq_source = DT_INST_IRQN(0),
	.irq_priority = DT_INST_IRQ(0, priority),
	.irq_flags = DT_INST_IRQ(0, flags),
};

static struct video_esp32_data esp32_data = {0};

DEVICE_DT_INST_DEFINE(0, video_esp32_init, NULL, &esp32_data, &esp32_config, POST_KERNEL,
		      CONFIG_VIDEO_INIT_PRIORITY, &esp32_driver_api);

VIDEO_DEVICE_DEFINE(esp32, DEVICE_DT_INST_GET(0), SOURCE_DEV(0));

static int video_esp32_cam_init_main_clock(void)
{
	int ret = 0;

	ret = pinctrl_apply_state(esp32_config.pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		printk("video pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Enable peripheral */
	if (!device_is_ready(esp32_config.clock_dev)) {
		return -ENODEV;
	}

	clock_control_on(esp32_config.clock_dev, esp32_config.clock_subsys);

	if (!esp32_config.cam_clk) {
		printk("No cam_clk specified\n");
		return -EINVAL;
	}

	if (ESP32_CLK_CPU_PLL_160M % esp32_config.cam_clk) {
		printk("Invalid cam_clk value. It must be a divisor of 160M\n");
		return -EINVAL;
	}

	/* Enable camera main clock output */
	cam_ll_select_clk_src(0, LCD_CLK_SRC_PLL160M);
	cam_ll_set_group_clock_coeff(0, ESP32_CLK_CPU_PLL_160M / esp32_config.cam_clk, 0, 0);

	return 0;
}

SYS_INIT(video_esp32_cam_init_main_clock, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

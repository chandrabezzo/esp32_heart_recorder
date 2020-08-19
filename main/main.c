#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "board.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "wav_encoder.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "equalizer.h"
#include <time.h>
#include <stdio.h>
#include <ds3231.h>

static const char *TAG = "E_STETOSKOP";

#define RECORD_TIME_SECONDS (60)
#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 21
#define SCL_GPIO 22
#endif

#define GREEN_LED GPIO_NUM_5
#define RED_LED GPIO_NUM_18

char *append(char *str1, char *str2){
	char* has_append;
	has_append = malloc(strlen(str1)+strlen(str2));
	strcpy(has_append, str1);
	strcat(has_append, str2);

	return has_append;
}

struct tm* ds3231_time()
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));

    time_t rawtime;
	struct tm *t;
	time(&rawtime);
	t = localtime(&rawtime);

	if (ds3231_get_time(&dev, t) != ESP_OK)
	{
		printf("Could not get time\n");
	}
	return t;
}

void record(struct tm * get_time){
	audio_pipeline_handle_t pipeline;
	audio_element_handle_t fatfs_stream_writer, i2s_stream_reader, wav_encoder, equalizer;

	esp_log_level_set("*", ESP_LOG_WARN);
	esp_log_level_set(TAG, ESP_LOG_INFO);

	ESP_LOGI(TAG, "[ 1 ] Mount sdcard");
	// Initialize peripherals management
	esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
	esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

	// Initialize SD Card peripheral
	audio_board_sdcard_init(set);

	ESP_LOGI(TAG, "[ 2 ] Start codec chip");
	audio_board_handle_t board_handle = audio_board_init();
	audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);

	ESP_LOGI(TAG, "[3.0] Create audio pipeline for recording");
	audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
	pipeline = audio_pipeline_init(&pipeline_cfg);
	mem_assert(pipeline);

	ESP_LOGI(TAG, "[3.1] Create fatfs stream to write data to sdcard");
	fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
	fatfs_cfg.type = AUDIO_STREAM_WRITER;
	fatfs_stream_writer = fatfs_stream_init(&fatfs_cfg);

	ESP_LOGI(TAG, "[3.2] Create i2s stream to read data from codec chip");
	i2s_stream_cfg_t i2s_cfg_read = I2S_STREAM_CFG_DEFAULT();
	i2s_cfg_read.type = AUDIO_STREAM_READER;
	i2s_stream_reader = i2s_stream_init(&i2s_cfg_read);

    equalizer_cfg_t eq_cfg = DEFAULT_EQUALIZER_CONFIG();
	int set_gain[] = { 20, 20, 20, 20, 13, 13, 13, 13, 13, 13, 20, 20, 20, 20, 13, 13, 13, 13, 13, 13};
	eq_cfg.set_gain = set_gain; // The size of gain array should be the multiplication of NUMBER_BAND and number channels of audio stream data. The minimum of gain is -20 dB.
	equalizer = equalizer_init(&eq_cfg);

	ESP_LOGI(TAG, "[3.3] Create wav encoder to encode wav format");
	wav_encoder_cfg_t wav_cfg = DEFAULT_WAV_ENCODER_CONFIG();
	wav_encoder = wav_encoder_init(&wav_cfg);

	ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
	audio_pipeline_register(pipeline, i2s_stream_reader, "i2s_reader");
	audio_pipeline_register(pipeline, equalizer, "equalizer");
	audio_pipeline_register(pipeline, wav_encoder, "wav");
	audio_pipeline_register(pipeline, fatfs_stream_writer, "file");

	ESP_LOGI(TAG, "[3.5] Link it together [codec_chip]-->i2s_stream-->wav_encoder-->fatfs_stream-->[sdcard]");
	audio_pipeline_link(pipeline, (const char *[]) {"i2s_reader", "equalizer", "wav", "file"}, 4);

	ESP_LOGI(TAG, "[3.6] Set up  uri (file as fatfs_stream, wav as wav encoder)");
	char timename[100];
	strftime(timename, sizeof(timename)-1, "%H%M%d", get_time);

	char * file_name = append("/sdcard/", timename);
	char * name_extension = append(file_name, ".wav");
	audio_element_set_uri(fatfs_stream_writer, name_extension);


	ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
	audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
	audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

	ESP_LOGI(TAG, "[4.1] Listening event from pipeline");
	audio_pipeline_set_listener(pipeline, evt);

	ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
	audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);


	ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
	audio_pipeline_run(pipeline);

	ESP_LOGI(TAG, "[ 6 ] Listen for all pipeline events, record for %d Seconds", RECORD_TIME_SECONDS);
	int second_recorded = 0;
	while (1) {
		audio_event_iface_msg_t msg;
		if (audio_event_iface_listen(evt, &msg, 1000 / portTICK_RATE_MS) != ESP_OK) {
			second_recorded ++;
			ESP_LOGI(TAG, "[ * ] Recording ... %d", second_recorded);
			if (second_recorded >= RECORD_TIME_SECONDS) {
				break;
			}
			continue;
		}

		/* Stop when the last pipeline element (i2s_stream_reader in this case) receives stop event */
		if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_reader
			&& msg.cmd == AEL_MSG_CMD_REPORT_STATUS
			&& (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
			ESP_LOGW(TAG, "[ * ] Stop event received");
			break;
		}
	}

	ESP_LOGI(TAG, "[ 7 ] Stop audio_pipeline");
	audio_pipeline_terminate(pipeline);

	audio_pipeline_unregister(pipeline, wav_encoder);
	audio_pipeline_unregister(pipeline, i2s_stream_reader);
	audio_pipeline_unregister(pipeline, equalizer);
	audio_pipeline_unregister(pipeline, fatfs_stream_writer);

	/* Terminal the pipeline before removing the listener */
	audio_pipeline_remove_listener(pipeline);

	/* Stop all periph before removing the listener */
	esp_periph_set_stop_all(set);
	audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

	/* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
	audio_event_iface_destroy(evt);

	/* Release all resources */
	audio_pipeline_deinit(pipeline);
	audio_element_deinit(fatfs_stream_writer);
	audio_element_deinit(i2s_stream_reader);
	audio_element_deinit(equalizer);
	audio_element_deinit(wav_encoder);
	esp_periph_set_destroy(set);
}

void app_main(void)
{
	// Get current time from Adafruit DS3231 RTC
	ESP_ERROR_CHECK(i2cdev_init());
	struct tm* get_time = ds3231_time();
	ESP_LOGI(TAG, "Current DateTime: %04d-%02d-%02d %02d:%02d:%02d\n", get_time->tm_year + 1900 /*Add 1900 for better readability*/, get_time->tm_mon + 1,
	                get_time->tm_mday, get_time->tm_hour, get_time->tm_min, get_time->tm_sec);
	ESP_ERROR_CHECK(i2cdev_done());

	// Setup gpio for 5mm LED
	gpio_pad_select_gpio(GREEN_LED);
	gpio_pad_select_gpio(RED_LED);
	gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	gpio_set_level(RED_LED, 0);
	gpio_set_level(GREEN_LED, 1);

	record(get_time);

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	gpio_set_level(GREEN_LED, 0);
	gpio_set_level(RED_LED, 1);
}

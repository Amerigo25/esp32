#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "sdkconfig.h"

#include <driver/i2s.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "sh1107.h" /* sh1107 driver*/
	
#define I2S_WS	GPIO_NUM_25 /* word select line*/
#define I2S_SD	GPIO_NUM_33 /* serial data line*/
#define I2S_SCK	GPIO_NUM_32 /* bit clock line*/
#define I2S_PORT I2S_NUM_0 /* i2s processor*/

#define I2C_SCL	19 /* i2c clock line*/
#define I2C_SDA	18 /* i2c data line*/

#define I2C_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

#define SAMPLE_RATE 16000
#define BUFFER_SIZE	256
#define SCREEN_WIDTH 128

#define WAVE_FREQ_HZ	100
#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)
#define PI	3.14159265
#define AMPLITUDE 32   // Amplitude of the sine wave
#define OFFSET 31.5 

int16_t* r_buff;
int16_t* draw_buff;
volatile int displayIndex = 0;
static SH1107_t dev; // oled handle

void init_i2s(void)
{
	i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_RX,
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};
	
	i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD};
	
	ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &i2s_mic_pins));
}

void init_oled (void)
{
	i2c_master_init(&dev,I2C_SDA,I2C_SCL,0);
	sh1107_init(&dev, 64, 128); /* width x height */
	sh1107_contrast(&dev, 0xff);
	sh1107_clear_screen(&dev, false);
	sh1107_direction(&dev, DIRECTION90);

}

void generateSineWave(uint8_t* buffer, int bufferSize, float frequency) {
    for (int i = 0; i < bufferSize; i++) {
        float angle = (float)i / (bufferSize - 1) * 2 * PI * frequency; // Calculate angle in radians
        buffer[i] = (uint8_t)(AMPLITUDE * sin(angle) + OFFSET); // Generate sine wave values
    }
}

/* 
	draw one pixel at position (x,y) of the screen, x in range [0,127], y in range [0,63]
	
*/
void sh1107_draw_pixel(SH1107_t* dev, int xpos, int ypos)
{
	int _seg = 63 - ypos;
	int page = (xpos / 8);
	int bit = (xpos % 8);
	uint8_t mask = 1 << bit;
	dev->_page[page]._segs[_seg] |= mask;
}

/* 
	draw one pixel at position (x,y) of the screen, x in range [0,127], y in range [0,63]
	
*/
void sh1107_draw_line(SH1107_t* dev, int xpos, int ypos)
{
	int _seg = 63 - ypos;
	int page = (xpos / 8);
	int bit = (xpos % 8);
	uint8_t mask = 1 << bit;
	for (int i=0; i<_seg; i++) {
		dev->_page[page]._segs[i] |= mask;
	}
}

uint8_t map (int16_t x) {
     return (uint8_t)((x+32768)*63/65534);
}

void sh1107_plot_audio(SH1107_t* dev, int16_t* buffer, int start)
{
	char x_char[8];
	char y_char[8];
	for (int x=0; x<SCREEN_WIDTH; x++) {
		// x->[0,127], y->[0,63]
		int i = (displayIndex+x)%SCREEN_WIDTH;
		uint8_t y = map(buffer[i]); // map function [-32768,32767] -> [0,63]
		sprintf(x_char,"x: %d",x%SCREEN_WIDTH);
		sprintf(y_char,"y: %d",y-31);	
		sh1107_display_text(dev,0,0,x_char,6,false);
		sh1107_display_text(dev,1,0,y_char,6,false);
		sh1107_draw_pixel(dev,x,y);
		sh1107_show_buffer(dev);
	}
}

/* read audio data task */
void readTask(void *args) {
	
	size_t r_bytes = 0;
	size_t r_samples = 0;	

	while(1) {
		// r_buff -> when passing an array to a function it decays to a pointer to is first element!					
		if (i2s_read(I2S_NUM_0, r_buff, BUFFER_SIZE*sizeof(int16_t), &r_bytes, 1000) == ESP_OK) {
			r_samples = r_bytes/sizeof(int16_t);
			for (int i = 0; i < r_samples; i++) {
				draw_buff[displayIndex] = r_buff[i];
				displayIndex = (displayIndex+1)%SCREEN_WIDTH;
			}
		}
	}
}

/* plot audio data task */
void drawTask(void *args) {
	
	while(1) {
		sh1107_clear_screen(&dev,false);
		sh1107_plot_audio(&dev,draw_buff,0);
		vTaskDelay(pdMS_TO_TICKS(20));		

	}
}

void app_main(void)
{			
		init_i2s();
		init_oled();
		
		r_buff = (int16_t *)malloc(BUFFER_SIZE*sizeof(int16_t));
		draw_buff = (int16_t *)malloc(128*sizeof(int16_t)); // circular buffer 

		xTaskCreatePinnedToCore(&readTask,"readTask",2048,NULL,1,NULL,0);
		xTaskCreatePinnedToCore(&drawTask,"drawTask",2048,NULL,1,NULL,1);

		
		//sh1107_fadeout(&dev);
		//fflush(stdout);
			
		//printf("Restarting now.\n");
		//esp_restart();
		
	
}

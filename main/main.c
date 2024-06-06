#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "driver/i2s_std.h"
#include "driver/i2c.h"
// #include "sh1107.h" /* sh1107 driver*/
	
#define I2S_WS	GPIO_NUM_25 /* word select line*/
#define I2S_SD	GPIO_NUM_33 /* serial data line*/
#define I2S_SCK	GPIO_NUM_32 /* bit clock line*/
#define I2S_PORT I2S_NUM_0 /* i2s processor*/

#define I2C_SCL	19 /* i2c clock line*/
#define I2C_SDA	18 /* i2c data line*/

#define I2C_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000

#define SAMPLE_RATE 8000
#define BITS_PER_SAMPLE	16
#define SAMPLE_SIZE	(BITS_PER_SAMPLE * 1024)
#define BYTE_RATE	(SAMPLE_RATE * (BITS_PER_SAMPLE / 8))


#define WAVE_FREQ_HZ	100
#define SAMPLE_PER_CYCLE (SAMPLE_RATE/WAVE_FREQ_HZ)
#define PI	3.14159265
#define AMPLITUDE 32   // Amplitude of the sine wave
#define OFFSET 31.5 


/*	I2S SETUP	*/
static i2s_chan_handle_t rx_handle;
static i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,   
            .bclk = I2S_SCK,
            .ws   = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
};
	
void generateSineWave(uint8_t* buffer, int bufferSize, float frequency) {
    for (int i = 0; i < bufferSize; i++) {
        float angle = (float)i / (bufferSize - 1) * 2 * PI * frequency; // Calculate angle in radians
        buffer[i] = (uint8_t)(AMPLITUDE * sin(angle) + OFFSET); // Generate sine wave values
    }
}
void generateCosWave(uint8_t* buffer, int bufferSize, float frequency) {
    for (int i = 0; i < bufferSize; i++) {
        float angle = (float)i / (bufferSize - 1) * 2 * PI * frequency; // Calculate angle in radians
        buffer[i] = (uint8_t)(AMPLITUDE * cos(angle) + OFFSET);		// Generate sine wave values
    }
}



void addSinousoids (uint8_t* buffer, int bufferSize, float* frequencies) 
{
	float increment = 2.0 * M_PI / bufferSize;
	for (int i = 0; i < bufferSize; i++) {
        float value = OFFSET;
        for (int j = 0; j < 4; j++) {
            value += AMPLITUDE * sin(frequencies[j] * increment * i );
        }
		if (value > 63) {
			value = 63;
		}
        // Scale the value to fit within 0-255 range
        buffer[i] = (uint8_t)(value);
	}
}
void sh1107_draw_pixel(SH1107_t* dev, int xpos, int ypos)
{
	int _seg = 63 - ypos;
	int page = (xpos / 8);
	int bit = (xpos % 8);
	uint8_t mask = 1 << bit;
	//wk0 = dev->_page[page]._segs[_seg];
	//byte = sh1107_rotate_byte(byte);
	dev->_page[page]._segs[_seg] |= mask;
}
	
void sh1107_plot_audio(SH1107_t* dev, uint8_t* buffer)
{
	for (int x=0; x<128; x++) {
		int y = buffer [x]; // int between 0 and 63
		sh1107_draw_pixel(dev,x,y);
		sh1107_show_buffer(dev);
	}
}


void app_main(void)
{
		
	/* initialize sh1107 */
	SH1107_t dev;
	i2c_master_init(&dev,I2C_SDA,I2C_SCL,0);
	sh1107_init(&dev, 64, 128); /* width x height */
	sh1107_contrast(&dev, 0xff);

	uint8_t sin [128];
	uint8_t cos [128];
	uint8_t add [128];

	 
	sh1107_clear_screen(&dev, false);
	
	/* initialize i2s microphone */	
	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
	ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));
	ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
	ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
	printf("we arrived till here\n"); 
	


	//init_microphone();
	
	
    while(1) {
		
		
		/* setting screen*/
		float f = 2;
		float freqs[4] = {1.0,2.0,3.0,4.0};

		/*
		sh1107_clear_screen(&dev, false);
		generateSineWave (sin,128,f);
		sh1107_plot_audio(&dev,sin);
		generateCosWave (cos,128,f);
		sh1107_plot_audio(&dev,cos);
		sh1107_clear_screen(&dev, false);
		//addSinousoids(add,128,freqs);
		sh1107_plot_audio(&dev,add);
		*/

		
		/*
		for (int i=0; i<128; i++) {
			for (int j=63;j>0; j--) {
				sh1107_draw_pixel(&dev,i,j);
			}
		}
		*/

		
		
		size_t r_bytes = 0;
		size_t r_samples = 0;

		// r_buff -> when passing an array to a function it decays to a pointer to is first element!
		uint16_t r_buff [2048];
		esp_err_t result = i2s_channel_read(rx_handle, r_buff, 2048, &r_bytes, 1000);
		if (result == ESP_OK) {
			r_samples = r_bytes/2;

			for (int i = 0; i < r_samples; i++) {
				
				if (i%100 == 0) {
				printf("%" PRId16 "\n",r_buff[i]);
				}
			}
		vTaskDelay(1);
				
			//printf("%.d\n",r_samples); // 8192 samples 16184 bytes
			//printf("%.3f\n",mean);
			// printf ("%d\n",r_samples); : 1024
			
			//vTaskDelay(1000 / portTICK_PERIOD_MS);
			//sh1107_plot_audio (&dev, r_buff, BUFFER_SIZE);
			//sh1107_display_text(&dev, 0, 0, "Reading from microphone...", 16, false);
		}
		
		else {
            sh1107_display_text(&dev, 0, 0, "Read failed!", 13, false);
		}
		
		

	        vTaskDelay(4);
		
		
		/*
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			sh1107_fadeout(&dev);
			fflush(stdout);
			
			printf("Restarting now.\n");
			esp_restart();
		*/
	}		
}

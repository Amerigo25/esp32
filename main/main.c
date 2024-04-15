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
#include "sh1107.h" /* sh1107 driver*/
	
#define I2S_WS	GPIO_NUM_25 /* word select line*/
#define I2S_SD	GPIO_NUM_27 /* serial data line*/
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


static uint16_t r_buff[64]; 
static i2s_chan_handle_t rx_handle = NULL;
static i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2C_NUM_0, I2S_ROLE_MASTER);


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
        buffer[i] = (uint8_t)(AMPLITUDE * cos(angle) + OFFSET); // Generate sine wave values
    }
}
void scrollSineWave(uint8_t* buffer, int bufferSize, int scrollAmount) {
    // Shift the entire buffer to the left by scrollAmount pixels
    for (int i = 0; i < bufferSize - scrollAmount; i++) {
        buffer[i] = buffer[i + scrollAmount];
    }
    
    // Generate new values for the scrolled portion of the buffer
    generateSineWave(buffer + (bufferSize - scrollAmount), scrollAmount, 1.0); // Adjust frequency as needed
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
	}
}


void app_main(void)
{
		
	/* initialize sh1107 */
	SH1107_t dev;
	i2c_master_init(&dev,I2C_SDA,I2C_SCL,0);
	sh1107_init(&dev, 64, 128); /* width x height */
	sh1107_contrast(&dev, 0xff);
	sh1107_clear_screen(&dev, false);

	uint8_t sin [128];
	uint8_t cos [128];

	uint8_t tri [128];

	 

	/* initialize i2s microphone 
	ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
	ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
	printf("fin qui, ci siamo arrivati\n"); 
	*/


	//init_microphone();
	
	
    while(1) {
		
		
		/* setting screen*/
		float f = 2;
		generateSineWave (sin,128,f);
		generateCosWave (cos,128,f);

		
		/*
		for (int i=0; i<128; i++) {
			for (int j=63;j>0; j--) {
				sh1107_draw_pixel(&dev,i,j);
			}
		}
		*/

		sh1107_plot_audio(&dev,sin);
		sh1107_plot_audio(&dev,cos);
		sh1107_show_buffer(&dev);
		vTaskDelay(4);
		
		
		//size_t r_bytes = 0;
		/* r_buff -> when passing an array to a function it decays to a pointer to is first element!*/
		/*
		esp_err_t result = i2s_channel_read(rx_handle, r_buff, SAMPLE_SIZE, &r_bytes, 1000);
		if (result == ESP_OK) {
			float mean = 0;
			r_samples = r_bytes/2;

			for (int16_t i = 0; i < r_samples; i++) {
				
				mean += r_buff[i];
				if (i%500 == 0) {
				printf("%" PRId16 "\n",r_buff[i]);
				}
			}
				
			mean /= r_samples;
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
		
		*/

	         		
		
			/*
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			sh1107_fadeout(&dev);
			fflush(stdout);
			
			printf("Restarting now.\n");
			esp_restart();
			*/
	}		
}

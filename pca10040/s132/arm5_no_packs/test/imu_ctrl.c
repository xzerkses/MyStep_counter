
#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_drv_timer.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "bmi160.h"
//#include "imu_ctrl.h"
//#include "arm_math.h"
#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(1000)

#define SPI_SS_PIN 22 
#define SPI_MISO_PIN 24
#define SPI_MOSI_PIN 23 
#define SPI_SCK_PIN 25 

#define INT_PIN 20

#define SPI_INSTANCE 0 // SPI instance index. We use SPI master 0  

#define NUM_TAPS 58 
#define BLOCK_SIZE 28 
#define BMI160_INT_LED 2
//// Declare a state array 
//static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1]; 

//float32_t B[NUM_TAPS];

		struct bmi160_dev sensor;
		//SPI instance 
		static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); 
		//Flag used to indicate that SPI instance completed the transfer 
		static volatile bool spi_xfer_done;
	  //Declare memory to store the 
		static uint8_t SPI_RX_Buffer[100]; // Allocate a buffer for SPI reads
		
		uint16_t step_count = 0;//stores the step counter value
		
void int_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
	{
			//int8_t bmi_data=
			//get_bmi160_fifo_data();
			nrf_gpio_pin_toggle(BMI160_INT_LED); 
		
			int8_t rslt = BMI160_OK;
			rslt = bmi160_read_step_counter(&step_count,  &sensor);
	}
static void gpio_init()
{
		ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    err_code = nrf_drv_gpiote_in_init(INT_PIN, &in_config, int_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT_PIN, true);
} 

//// Declare an instance for the low-pass FIR filter 
//arm_fir_instance_f32 fir_lpf;  

///** 
// * Function for initializing the FIR filter instance 
// */ 
//void dsp_config() { 
// 
// // Note that the init function requires the address of the coefficient table B as 
//	//an input 
//		arm_fir_init_f32(&fir_lpf, NUM_TAPS, (float32_t *)&B[0], &firStateF32[0], 
//BLOCK_SIZE); 
// 
//} 
	
		

//		//Declare memory to store the 
//		static uint8_t SPI_RX_Buffer[100]; // Allocate a buffer for SPI reads

//		uint8_t fifo_buffer[200];
//		struct bmi160_fifo_frame fifo_frame;

//		// 200 bytes -> ~7bytes per frame -> ~28 data frames 
//		struct bmi160_sensor_data gyro_data[28];
//		

/** 
	 * SPI user event handler. 
	 */ 
	void spi_event_handler(nrf_drv_spi_evt_t const * p_event, 
	 void * p_context) 
	{ 
			spi_xfer_done = true; // Set a flag when transfer is done 
	} 

/** 
 * Function for setting up the SPI communication. 
 */ 
uint32_t spi_config() 
{ 
	 uint32_t err_code; 
	 
	 // Use nRF's default configurations 
	 nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG; 
	 
	 // Define each GPIO pin 
	 spi_config.ss_pin = SPI_SS_PIN; 
	 spi_config.miso_pin = SPI_MISO_PIN; 
	 spi_config.mosi_pin = SPI_MOSI_PIN; 
	 spi_config.sck_pin = SPI_SCK_PIN; 
	 
	 // Initialize the SPI peripheral and give it a function pointer to 
	 // it’s event handler 
	 err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL); 
	 
	 return err_code; 
} 



/** 
 * Function for writing to the BMI160 via SPI. 
 */ 
int8_t bmi160_spi_bus_write(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, 
uint16_t cnt) 
{ 
		 spi_xfer_done = false; // set the flag down during transfer 
		 
		 int32_t error = 0; 
		 
		 // Allocate array, which lenght is address + number of data bytes to be sent 
		 uint8_t tx_buff[cnt+1]; 
		 
		 uint16_t stringpos; 
		 
		 // AND address with 0111 1111; set msb to '0' (write operation) 
		 tx_buff[0] = reg_addr & 0x7F; 
 
		 for (stringpos = 0; stringpos < cnt; stringpos++) { 
				tx_buff[stringpos+1] = *(reg_data + stringpos); 
		 } 
		 // Do the actual SPI transfer 
		 nrf_drv_spi_transfer(&spi, tx_buff, cnt+1, NULL, 0); 
		 
		 while (!spi_xfer_done) {}; // Loop until the transfer is complete 
		 
		 return (int8_t)error; 
} 

/** 
 * Function for reading from the BMI160 via SPI. 
 */ 
int8_t bmi160_spi_bus_read(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, 
uint16_t len) 
{ 
	 spi_xfer_done = false; // set the flag down during transfer 
	 
	 int32_t error = 0; 
	 uint8_t tx_buff = reg_addr | 0x80; // OR address with 1000 0000; Read -> set msb to '1'; 
	 uint8_t * rx_buff_pointer; 
	 uint8_t stringpos; 
	 
	 rx_buff_pointer = (uint8_t *) (SPI_RX_Buffer); 
	 
	 // Do the actual SPI transfer 
	 nrf_drv_spi_transfer(&spi, &tx_buff, 1, rx_buff_pointer, len+1); 
	 
	 while (!spi_xfer_done) {} // Loop until the transfer is complete 
	 
	 // Copy received bytes to reg_data 
	 for (stringpos = 0; stringpos < len; stringpos++) 
	 *(reg_data + stringpos) = SPI_RX_Buffer[stringpos + 1]; 
	 
	 return (int8_t)error; 
} 

int8_t sensor_init() 
{ 
	 int8_t rslt = BMI160_OK; 
	 //gpio_init();
	 sensor.id = 0; // We use SPI so id == 0 
	 sensor.interface = BMI160_SPI_INTF; 
	 // Give the driver the correct interfacing functions 
	 sensor.read = bmi160_spi_bus_read; 
	 sensor.write = bmi160_spi_bus_write; 
	 sensor.delay_ms = nrf_delay_ms; 
	 
	 // Initialize the sensor and check if everything went ok 
	 rslt = bmi160_init(&sensor); 
	 
//	 return rslt; 
//} 



//int8_t bmi160_config()
//{
//		int8_t rslt = BMI160_OK;
		
		/* Select the Output data rate, range of accelerometer sensor */
		
		sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
		sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
		sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
		
		/* Select the power mode of accelerometer sensor */
		sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
		
//		/* Select the Output data rate, range of Gyroscope sensor */
//		sensor.gyro_cfg.odr = BMI160_GYRO_ODR_25HZ;
//		sensor.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
//		sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

//		/* Select the power mode of Gyroscope sensor */
//		sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

		/* Set the sensor configuration */
		rslt = bmi160_set_sens_conf(&sensor);

////		//fifo 
////		fifo_frame.data=fifo_buffer;
////		fifo_frame.length=200;
////		sensor.fifo=&fifo_frame;

//		//fifo settings
//		rslt=bmi160_set_fifo_config(BMI160_FIFO_GYRO, BMI160_ENABLE,&sensor);

		struct bmi160_int_settg int_config;

		//Interrupt channel/pin 1
		int_config.int_channel = BMI160_INT_CHANNEL_1; 

		/* Select the Interrupt type */
		int_config.int_type = BMI160_STEP_DETECT_INT;// Choosing Step Detector interrupt

		

		// Enabling interrupt pins to act as output pin 
		int_config.int_pin_settg.output_en = BMI160_ENABLE; 
		
		// Choosing push-pull mode for interrupt pin 
		int_config.int_pin_settg.output_mode = BMI160_DISABLE; 
		// Choosing active high output 
		int_config.int_pin_settg.output_type = BMI160_ENABLE; 
		// Choosing edge triggered output 
		int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE; 
		// Disabling interrupt pin to act as input 
		int_config.int_pin_settg.input_en = BMI160_DISABLE; 
		// Non-latched output 
		int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE; 
		/* Select the Step Detector interrupt parameters, Kindly use the recommended settings for step detector */
		int_config.int_type_cfg.acc_step_detect_int.step_detector_mode = BMI160_STEP_DETECT_NORMAL;
		int_config.int_type_cfg.acc_step_detect_int.step_detector_en = BMI160_ENABLE;// 1-enable, 0-disable the step detector

		// Set interrupt configurations 
		rslt = bmi160_set_int_config(&int_config, &sensor); 
		
		rslt = BMI160_OK;
		uint8_t step_enable = 1;//enable the step counter
		//set step counter
		rslt = bmi160_set_step_counter(step_enable,  &sensor);
		return rslt;
}

	/** a
	int8_t get_bmi160_step_counts()
	{
		int8_t rslt = BMI160_OK;
		

		rslt = bmi160_read_step_counter(&step_count,  &sensor);
		
	
	
//	/** 
//	* Function for reading FIFO data 
//	*/ 
//	int8_t get_bmi160_fifo_data() 
//	{ 
//		 int8_t rslt = BMI160_OK; 
//		 uint8_t gyro_frames_req = 28; 
//		 // Read the fifo buffer using SPI 
//		 rslt = bmi160_get_fifo_data(&sensor); 
//		 // Parse the data and extract 28 gyro frames 
//		 rslt = bmi160_extract_gyro(gyro_data, &gyro_frames_req, &sensor); 
//		 
//		// Copy the contents of each axis to a FIR input buffer 
//		for (uint8_t i = 0; i < gyro_frames_req; i++) { 
//			 gyro_x_in_buf[gyro_frames_req*block_cnt + i] = gyro_data[i].x; 
//			 gyro_y_in_buf[gyro_frames_req*block_cnt + i] = gyro_data[i].y; 
//			 gyro_z_in_buf[gyro_frames_req*block_cnt + i] = gyro_data[i].z; 
//		} 
//		 
//		// Increase block count after each sensor read 
//			block_cnt++; 
//		 
//		// After 5 reads the buffer is almost full and the data is ready to be processed 
//		if (block_cnt == 5) { 
//				compute_fir(); 
//		} 
//		
//		 return rslt; 
//	} 
	
		
	
	

	

	


void bmi160_stepp_meas_init(void)
{
		//spi_init();
		gpio_init();
		spi_config();
		sensor_init();
}
	
	
	
	


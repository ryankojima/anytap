/*
Task around accelerometer
 */

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "acc_task.h"

#define ACC_INT1 15
#define ACC_INT2 32
#define GPIO_LED 13

#define V_REF   1100
#define ADC_BATTERY (ADC1_CHANNEL_7)


#define LS_OUT 12

spi_device_handle_t spi;
static const char *TAG = "ACC";
void (*clickCallback)(void *) = (void *)0;
bool lockDeepSleepStatus = false;
RTC_DATA_ATTR sleep_cause lastSleepCause;

int acc_wake_count=0;

int event_count=0;


void start_bell(void);
void stop_bell(void);
uint32_t read_battery(void);

void lockDeepSleep(void)
{
  lockDeepSleepStatus=true;
}
void unlockDeepSleep(void)
{
  lockDeepSleepStatus=false;
}
bool isLockDeepSleep(void){
  return lockDeepSleepStatus;
}
void set_acc_wake_count(int wc){
  acc_wake_count=wc;
}

uint8_t acc_spi_read(spi_device_handle_t spi, const uint8_t addr)
{
    esp_err_t ret;
    spi_transaction_t t;
	uint8_t buf;
	uint8_t rxBuf=0;
	buf=addr | 0x80;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=16;                     //Command is 16 bits total
    //t.tx_buffer=&buf;               //The data is the cmd itself
	t.tx_data[0]=buf;
	//t.rxlength=8;
	//t.rx_buffer=&rxBuf;
	t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;

    ret=spi_device_queue_trans(spi, &t,portMAX_DELAY);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

	spi_transaction_t *tp;
	tp=&t;
	ret=spi_device_get_trans_result(spi,&tp,portMAX_DELAY);
    assert(ret==ESP_OK);            //Should have had no issues.
	
	rxBuf=(uint8_t)t.rx_data[1];
	//ESP_LOGI(TAG,"Data read=%x",rxBuf);
	return rxBuf;
}
uint16_t acc_spi_read16(spi_device_handle_t spi, const uint8_t addr)
{
    esp_err_t ret;
    spi_transaction_t t;
	uint8_t buf;
	uint16_t rxBuf;
	buf=addr | 0xC0;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=24;                     //Command is 16 bits total
    //t.tx_buffer=&buf;               //The data is the cmd itself
	t.tx_data[0]=buf;
	//t.rxlength=16;
	//t.rx_buffer=&rxBuf;
	t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
	rxBuf=*(uint16_t *)(&t.rx_data[1]);
	//ESP_LOGI(TAG,"Data read=%x",rxBuf);
	return rxBuf;
}
void acc_spi_read48(spi_device_handle_t spi, const uint8_t addr,uint16_t *rxBuf)
{
    esp_err_t ret;
    spi_transaction_t t;
	uint8_t buf[7];
	uint8_t inRxBuf[7];
	uint16_t *inRx16=(uint16_t *)&inRxBuf[1];
	memset(buf,0,7*sizeof(uint8_t));
	buf[0]=addr | 0xC0;
	
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=56;
    t.tx_buffer=buf;               //The data is the cmd itself
	//t.tx_data[0]=buf;
	//t.rxlength=48;
	t.rx_buffer=(void *)inRxBuf;
	t.flags = 0;//SPI_TRANS_USE_TXDATA;


	//ret=spi_device_transmit(spi, &t);  //Transmit!

    spi_transaction_t *ret_trans;
    //ToDo: check if any spi transfers in flight
    ret=spi_device_queue_trans(spi, &t, portMAX_DELAY);
	assert(ret==ESP_OK);

    ret=spi_device_get_trans_result(spi, &ret_trans, portMAX_DELAY);
	assert(ret==ESP_OK);

    if(ret_trans != &t){
	  ESP_LOGI(TAG,"ret_trans is not equal to trans_desc.");
	}
	

	assert(ret==ESP_OK);            //Should have had no issues.

	for(int i=0;i < 3;i++){ rxBuf[i]=inRx16[i]; }
	
	//rxBuf=*(uint16_t *)t.rx_data;
	//ESP_LOGI(TAG,"Data read=%x",rxBuf);
	//return rxBuf;
}
void acc_spi_write(spi_device_handle_t spi, const uint8_t addr, const uint8_t data) 
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=16;              
    t.tx_data[0]=addr;
    t.tx_data[1]=data;
	t.flags = SPI_TRANS_USE_TXDATA;
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}


void adxl_write(spi_device_handle_t spi, const ADXL345_Address addr, const uint8_t data)
{
  acc_spi_write(spi,(uint8_t)addr,data);
}
uint8_t adxl_read(spi_device_handle_t spi, const ADXL345_Address addr)
{
  return acc_spi_read(spi,(uint8_t)addr);
}
uint16_t adxl_read16(spi_device_handle_t spi, const ADXL345_Address addr)
{
  return acc_spi_read16(spi,(uint8_t)addr);
}

void adxl_read48(spi_device_handle_t spi, const ADXL345_Address addr, uint16_t *rxBuf)
{
  acc_spi_read48(spi,(uint8_t)addr, rxBuf);
}

void init_acc(spi_device_handle_t spi){

  uint8_t devid;
  devid=adxl_read(spi,DEVID); // Connection checking by reading DEVID
  ESP_LOGI(TAG,"DEVID = %x",devid);
  
  assert(devid == 0xe5);
  
  adxl_write(spi,INT_ENABLE,0x00); //Disable all interrupts first
  adxl_write(spi,POWER_CTL,0x0); 
  
  
  adxl_write(spi,THRESH_TAP,0x06);
  adxl_write(spi,DUR,0x10);
  adxl_write(spi,Latent,0xA0);
  adxl_write(spi,Window,0xf0);
  adxl_write(spi,THRESH_ACT,0x08);
  adxl_write(spi,THRESH_INACT,0x2);
  adxl_write(spi,TIME_INACT,0x05);
  adxl_write(spi,ACT_INACT_CTL,0xf6);
  adxl_write(spi,TAP_AXES,0x6);

  
  adxl_write(spi,BW_RATE,0xa); //Data rate to 25Hz
  adxl_write(spi,FIFO_CTL,0x00); //Stream mode
  adxl_write(spi,INT_MAP,0x10);
  


  uint8_t val;
  /*
  //clear all interrupts during sleep;
  uint16_t data[3];
  while(1){
	val=adxl_read(spi,INT_SOURCE);
	ESP_LOGI(TAG,"Start up loop : Cuurent INT_SOURCE=%x",val);

	if((val&0xa0) != 0){
	  adxl_read48(spi,DATAX0, data);
	}else
	  break;
  }
  */

  
  adxl_write(spi,INT_ENABLE,0xb8);
  adxl_write(spi,POWER_CTL,0x28);//0x32);

  val=adxl_read(spi,INT_SOURCE);
  ESP_LOGI(TAG,"Start up loop: Cuurent INT_SOURCE=%x",val);
  
  devid=adxl_read(spi,DEVID); // Connection checking by reading DEVID
  ESP_LOGI(TAG,"DEVID = %x",devid);
  assert(devid == 0xe5);
  
	  gpio_set_level(GPIO_LED, 0);

}

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle click_evt_queue = NULL;

static void IRAM_ATTR tap_handler(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
bool int_inprocess = false;

void acc_interrupt_task(void* arg)
{
    uint32_t io_num;
	uint16_t data[3];
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
		  int_inprocess=true;
		  uint8_t event_type;
		  do{
			event_type = adxl_read(spi,INT_SOURCE);
			//adxl_read(spi,DATAX0); // dummy read
			if((event_type & 0x10) != 0){
			  ESP_LOGI(TAG,"Active event detected.%d",io_num);
			}
			if((event_type & 0x60) != 0){
			  
			  ESP_LOGI(TAG,"Double/Single tap detected.evt:%x",event_type);
			  //if(acc_wake_count == 0){
			  //ESP_LOGI(TAG,"Initial boot. Event ignored");
			  //}else
			  if(event_count > 0){
				ESP_LOGI(TAG,"Second event. Event ignored");
			  }else{
				xQueueSendToBack(click_evt_queue,&io_num,0);
				event_count++;
			  }
			}
			if((event_type & 0x08) != 0){
			  ESP_LOGI(TAG,"InActive event detected.");

			  int ct=0;
			  while(lockDeepSleepStatus){
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				ESP_LOGI(TAG, "Lock detected. Waiting...");
				ct++;
				if(ct == 5) break; // timeout for response
			  }


			  lastSleepCause = INACTIVE;
			  adxl_write(spi,INT_ENABLE,0x10); //disable all interrupts except Activity
			  //adxl_write(spi,BW_RATE,0x7); // reduce data rate
			  
			  
			  
			  
			  ESP_LOGI(TAG, "Entering deep sleep...");
			  esp_sleep_enable_ext0_wakeup(ACC_INT2,1);
			  esp_sleep_enable_timer_wakeup(1000ULL*1000*60*60*24); //wake up once a day to check battery
			  esp_deep_sleep_start();
			 
			}
			if((event_type & 0x80) != 0){
			  adxl_read48(spi,DATAX0, data);
			  //ESP_LOGI(TAG,"Read data=%x,%x,%x : Event ID=%x, IO : %d",data[0],data[1],data[2],event_type,io_num);
			}else{
			  //			ESP_LOGI(TAG,"Event : %x, IO: %d",event_type,io_num);
			}

		  }while((event_type) != 0);
		  
          //  printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
		  int_inprocess=false;
        }
    }
}

void click_task(void *arg){
  uint32_t io_num;
  for(;;) {
	if(xQueueReceive(click_evt_queue, &io_num, portMAX_DELAY)) {
	  gpio_set_level(GPIO_LED, 1);


	  
	  if(clickCallback){
		device_event_param event;
		event.event_type = CLICK;
		clickCallback((void *)&event);
	  }
	  start_bell();
	  vTaskDelay(100 / portTICK_PERIOD_MS);
	  stop_bell();
	  vTaskDelay(100 / portTICK_PERIOD_MS);
	  start_bell();
	  vTaskDelay(100 / portTICK_PERIOD_MS);
	  stop_bell();
	  vTaskDelay(200 / portTICK_PERIOD_MS);

	  gpio_set_level(GPIO_LED, 0);



	}
  }
}

esp_adc_cal_characteristics_t characteristics;

uint32_t read_battery(void){
    uint32_t voltage;
	voltage = adc1_to_voltage(ADC_BATTERY, &characteristics);
	ESP_LOGI(TAG,"Battery mV=%d",voltage*2);
	//printf("%d mV\n",voltage*2);
	//vTaskDelay(pdMS_TO_TICKS(1000));
	return voltage*2;
}

void init_adc(void){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_BATTERY, ADC_ATTEN_DB_11);
    esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, &characteristics);
}



ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel[1];

void start_bell(void){
  ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 4000);
  ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
}
void stop_bell(void){
  ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, 0);
  ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
}

void init_loud_speaker(void){
  ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
  ledc_timer.freq_hz = 500;
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.timer_num = LEDC_TIMER_0;
	
  ledc_timer_config(&ledc_timer);
  
  ledc_channel[0].channel    = LEDC_CHANNEL_0;
  ledc_channel[0].duty       = 0;
  ledc_channel[0].gpio_num   = 12;
  ledc_channel[0].speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_channel[0].timer_sel  = LEDC_TIMER_0;

	ledc_channel_config(&ledc_channel[0]);

    // Initialize fade service.
    ledc_fade_func_install(0);
	

}


void init_io(void){
  rtc_gpio_deinit(GPIO_NUM_4); // In case it comes from deep sleep

  gpio_config_t io_conf;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    //bit mask of the pins
    io_conf.pin_bit_mask = (1ULL << ACC_INT1) | (1ULL << ACC_INT2) ;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    //io_conf.pull_up_en = 1;
    gpio_config(&io_conf);


    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << GPIO_LED);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


	//install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(ACC_INT1, tap_handler, (void*) ACC_INT1);
    gpio_isr_handler_add(ACC_INT2, tap_handler, (void*) ACC_INT2);


	init_loud_speaker();
	init_adc();
	
}

void set_click_callback(void *callback){
  clickCallback = callback;
}

void monitor_acc_task(void *pvParameters){
  //spi_device_handle_t spi;
  esp_err_t ret;
  spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1*1000*1000,               //Clock out at 1 MHz
        .mode=3,                                //SPI mode 3
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
		//.address_bits=6,
		//.command_bits=8
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

  //Initialize the SPI bus
  ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  assert(ret==ESP_OK);
  //Attach the SPI device
  ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  assert(ret==ESP_OK);

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  
  click_evt_queue = xQueueCreate(10, sizeof(uint32_t));

	init_acc(spi);

  init_io();
  xTaskCreate(&acc_interrupt_task, "acc_interrupt_task", 8192, NULL, 10, NULL);
  xTaskCreate(&click_task, "click_task", 8192, NULL, 10, NULL);

  //make one event, anyway (testing)
  //uint32_t io_num=0;
  //xQueueSendToBack(click_evt_queue,&io_num,0);
  
  uint32_t voltage;
  voltage = read_battery();
  if(voltage <= 3650){
	if(clickCallback){
	  device_event_param event;
	  event.event_type = BATTERY;
	  event.battery_mv = voltage;
	  clickCallback((void *)&event);
	}
  }
  

  
  
  //  uint16_t data[3];
  uint8_t val,int_source;
  while(1){

			//val=adxl_read(spi,DEVID);
			//ESP_LOGI(TAG,"Cuurent DEVID=%x",val);
			if(!int_inprocess){
			  val=adxl_read(spi,INT_SOURCE);
			  int_source=val;
			  ESP_LOGI(TAG,"Cuurent INT_SOURCE=%x",val);
			  int num=21;
			  if((int_source&0x80) != 0){
				ESP_LOGI(TAG,"Event Dispatched by mainloop");
				xQueueSendFromISR(gpio_evt_queue, &num, NULL);
			  }
			}

			//val=adxl_read(spi,FIFO_STATUS);
			//ESP_LOGI(TAG,"Cuurent FIFO_STATUS=%x",val);

			//ESP_LOGI(TAG,"Waiting in monitor task");
			//adxl_read(spi,0);
			//adxl_read48(spi,DATAX0, data);
			//ESP_LOGI(TAG,"Read data=%x,%x,%x",data[0],data[1],data[2]);
            vTaskDelay(500 / portTICK_PERIOD_MS);
  }

}



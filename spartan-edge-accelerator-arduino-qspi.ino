#include <driver/spi_master.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "rom/ets_sys.h"

#include "esp_system.h"
#include <time.h>
#include <sys/time.h>

#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include <stdio.h>
#include <string.h>


#define ESP_INTR_FLAG_DEFAULT 0
SemaphoreHandle_t xSemaphore = NULL;

spi_transaction_t trans;
esp_err_t ret;
spi_device_handle_t spi;

int interrupt_count = 0;
void IRAM_ATTR vblank_handler() {
  //Serial.println("interrupt!");
  xSemaphoreGiveFromISR(xSemaphore, NULL);
  //Serial.println("interrupt complete!");
}
int sendQSPI = 0;

void qspi_task(void* arg) {
  while(1) {
    //Serial.print("*");
    if (xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE && sendQSPI) {
      //Serial.println("Semaphore and sendQSPI triggered.");
      interrupt_count++;
      spi_device_transmit(spi, &trans);
      sendQSPI = 0;
    }
  }
}

char time_string[] = "hello"; //[255];
uint8_t data[255];
int tick_count;
time_t rawtime;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Launching...");
  //settimeofday()
  
  struct tm * timeinfo;

  spi_bus_config_t buscfg;
    buscfg.miso_io_num = GPIO_NUM_19;
    buscfg.mosi_io_num = GPIO_NUM_23;
    buscfg.sclk_io_num = GPIO_NUM_18;
    buscfg.quadwp_io_num = GPIO_NUM_22;
    buscfg.quadhd_io_num = GPIO_NUM_21;
    buscfg.max_transfer_sz= 16*320*2+8;
    buscfg.flags = 0; //SPICOMMON_BUSFLAG_IOMUX_PINS
  
  spi_device_interface_config_t devcfg;
    devcfg.clock_speed_hz = 10*1000*1000; //80
    devcfg.mode = 0;
    devcfg.spics_io_num = 5; // previous example had -1 and did cs manually
    devcfg.queue_size = 7; //We want to be able to queue 7 transactions at a time
    //.address_bits = 16,
    devcfg.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY; //SPI_DEVICE_NODUMMY
  //Initialize the SPI bus
  ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
  if(ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add QSPI device");
    return;
  }
  ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
  if(ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to add QSPI device");
    return;
  }
  /* Transaction to read a register */
  uint16_t addr = 0x21C0;//0x21C0;
    data[0] = (uint8_t)(addr >> 8);
    data[1] = (uint8_t)(addr & 0xFF);
    uint32_t reg_data = 0xFF00FF00;

    //Send reg addr and data
    memset(&trans, 0, sizeof(trans)); // Zero out the transaction
    trans.tx_buffer = &data; //point to user buffer for Tx data
    trans.rxlength = 0;
    trans.length = 2*8;
    trans.flags = 0;
    trans.rx_buffer = NULL;
    trans.flags |= SPI_TRANS_MODE_QIO;

    // create the binary semaphore
  	xSemaphore = xSemaphoreCreateBinary();

    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_INPUT);
    gpio_set_intr_type(GPIO_NUM_27, GPIO_INTR_POSEDGE);
    xTaskCreate(qspi_task, "qspi_task", 2048, NULL, 10, NULL);
    //gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //gpio_isr_handler_add(GPIO_NUM_27, vblank_handler, NULL);
    attachInterrupt(digitalPinToInterrupt(GPIO_NUM_27), vblank_handler, FALLING);

    struct timeval nowtime;
    nowtime.tv_sec = 1652416348;
}

void loop() {
    //settimeofday(&nowtime, 0);
while(1) {
    //for (int i = 1; i >= 0; i--) {
    vTaskDelay(3.5);// / portTICK_PERIOD_MS);
    //}
    //time (&rawtime);
    //timeinfo = setTime(rawtime);
    char strftime_time[255];
    unsigned long millis_time = esp_timer_get_time();
    int str_length = 0; //24;
    //strftime(strftime_time, sizeof(strftime_time-1), "%I:%M:%S %p", timeinfo);
    str_length += snprintf(strftime_time, sizeof(strftime_time)-1, " %lu", millis_time);
    char* asc_time = strftime_time;
    // char* asc_time = "hello hello hello hello ";
    //char* asc_time = asctime(timeinfo);
    // int str_length = 24;
    // Serial.println(asc_time);
    for (int i = 0; i < str_length; i++) {
      char translated;
      if (asc_time[i] >= 'a' && asc_time[i] <= 'z') {
        data[2+i*2] = 0x10;
        data[2+i*2+1] = asc_time[i] - 97;
      }
      else if (asc_time[i] >= 'A' && asc_time[i] <= 'Z') {
        data[2+i*2] = 0x10;
        data[2+i*2+1] = asc_time[i] - 65;
      }
      else if (asc_time[i] >= '0' && asc_time[i] <= '9') {
    	data[2+i*2] = 0x10;
	    data[2+i*2+1] = asc_time[i] +26 - 48;
      }
      else if (asc_time[i] == ' ') {
      	data[2+i*2] = 0x10;
  	    data[2+i*2+1] = 0x78;
      }
      else if (asc_time[i] == ':' || asc_time[i] == '/') {
		data[2+i*2] = 0x10;
		data[2+i*2+1] = 0x27;
      }
    }
    trans.length = (2+str_length*2)*8;
//    trans.length = 10*8;
    sendQSPI = 1;
    //Serial.println("interrupt_count %d", interrupt_count);
    //tick_count = (tick_count+1)%10;
    //data[3] = 26+tick_count;
    interrupt_count = 0;
  }


//  esp_restart();
}

#include "contiki.h"
#include "dev/sht11.h"
#include "puppet-temp-hum.h"
#include <stdio.h>
#define PRINTF(d,...) if((x)){printf("DEBUG:",__VA_ARGS__);}

PROCESS(puppet_temp_hum, "Temperature & Humidity Reader Process");

static temp_hum_callback sensor_cbk = NULL;

void set_temp_hum_callback(temp_hum_callback callback){
  sensor_cbk = callback;
}

static
unsigned read_temp(void){
  return (unsigned)(-39.60 + 0.01*sht11_temp());
}

static
unsigned read_hum(void){
  static unsigned rh;
  rh = sht11_humidity();
  return (unsigned)(-4 + 0.0405*rh - 2.8e-6*(rh*rh));
}

int read_temp_hum(temp_hum_reading* buf){
  //if buf is acutally supplied, assume synchronous version
  //of temp and hum reader is intended.
  if(buf != NULL){
    buf->temperature = read_temp();
    buf->humidity = read_hum();
    return 1;
  }
  //spawn process to manage continuous reading of sensors
  //but first kill any other existing processes.
  if(sensor_cbk != NULL){
    process_exit(&puppet_temp_hum);
    process_start(&puppet_temp_hum, NULL);
  }else{
    PRINTF(1,"No callback function defined");
  }
  return 1;
}

PROCESS_THREAD(puppet_temp_hum, ev, data)
{
  static struct etimer et;
  PROCESS_BEGIN();
  sht11_init();
  
  #ifndef TEMP_HUM_READING_DURATION
  #define TEMP_HUM_READING_DURATION 60
  #endif
  for(etimer_set(&et,CLOCK_SECOND*READING_DURATION);;etimer_reset(&et)){
    PROCESS_YIELD(); //Yield until timer expires.
    sensor_cbk(read_temp(),read_hum());
  }
  PROCESS_END();
}

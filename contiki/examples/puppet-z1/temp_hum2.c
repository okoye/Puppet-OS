#include "contiki.h"
#include "puppet-temp-hum.h"
#include <stdio.h>

PROCESS(test_temp_hum, "Humidity and Temp Reader");
AUTOSTART_PROCESSES(&test_temp_hum);

int callBack(unsigned temp, unsigned hum){
  printf("Callback called successfully with %u %u",temp,hum);
}

PROCESS_THREAD(test_temp_hum, ev, data)
{
  PROCESS_BEGIN();
  set_temp_hum_callback(*callBack);
  read_temp_hum(NULL);

}

#include "contiki.h"
#include "dev/sht11.h"

#include <stdio.h>

PROCESS(test_sht11_process, "SHT Humidity & Temp Test");
AUTOSTART_PROCESSES(&test_sht11_process);

PROCESS_THREAD(test_sht11_process, ev, data)
{
  static unsigned rh;
  static struct etimer et;
  PROCESS_BEGIN();
  sht11_init();

  for(etimer_set(&et,CLOCK_SECOND*30);;etimer_reset(&et)){
    PROCESS_YIELD();
    printf("Temperature: %u degree Celsius\n",
      (-39.60 + 0.01 * sht11_temp()));
    rh = sht11_humidity();
    printf("Relative Humidity: %u%%\n",
      (unsigned)(-4 + 0.0405*rh - 2.8e-6*(rh*rh)));
  }
  PROCESS_END();
}

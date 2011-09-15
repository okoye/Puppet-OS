/**
 *Licensing & Copyright info
 *@author: Chuka
 *@email: chuka@puppetme.com
 */

#include <contiki.h>
#include "z1.h"
#include "rest.h"
#include "ptemperature.h"
#include "pcommon.h"

#define PRINTF(...) printf(__VA_ARGS__)

AUTOSTART_PROCESSES(&z1_test_device);

PROCESS_THREAD(z1_test_device, ev, data)
{
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  //Network Initialization Routines
  set_node_addresses();
  PRINTF("Started Z1 main\n");
  print_node_addresses();
  
  //Program Initialization Routines
  rest_init();
  ptemperature_initialize();

  PROCESS_END();
}

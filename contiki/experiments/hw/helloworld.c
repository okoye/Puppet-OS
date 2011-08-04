#include "contiki.h"

PROCESS(helloworldprocess, "Hello World Process");
AUTOSTART_PROCESSES(&helloworldprocess);

PROCESS_THREAD(helloworldprocess, event, data)
{
  PROCESS_BEGIN();
  printf("Hellow world from Contiki OS, Puppet Technology rocks!\n");
  PROCESS_END();
}

/**
 *Licensing & Copyright info
 *@author: Chuka
 *@email: chuka@puppetme.com
 */

#include <stdlib.h>
#include <string.h>
#include <contiki.h>
#include "z1.h"
#include "net/uip-debug.h"

PROCESS(z1_test_device, "Z1 Test Device");
AUTOSTART_PROCESSES(&z1_test_device);

static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}

static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 Addresses: ");
  for(i=0, i<UIP_DS6_ADDR_NB; i++){
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
      (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)){
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
    }
  }
}

PROCESS_THREAD(z1_test_device, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();

  PRINTF("Started Z1 main\n");

  print_local_addresses();

  //TODO: Initialize temperature

  PROCESS_END();
}

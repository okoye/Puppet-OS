/*
 *Licensing information & Copyright info
 *@author: Chuka
 *@email: chuka@puppetme.com
 */

#include <stdlib.h>
#include <string.h>
#include <contiki.h>
#include "ptemperature.h"
#include "rest.h"
#include "buffer.h"
/*******************************************
    Temperature Specific Include Files
********************************************/
#if PLATFORM_HAS_SHT11
#include "dev/temperature-sensor.h"
#endif

/*******************************************
              Some Macros
********************************************/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define SERVER_NODE(ipaddr) uip_ip6addr(ipaddr, 0xaaaa,0,0,0,0,0,0,1);

char outputBuffer[MAX_PAYLOAD_LEN];
static char* proxy_uri = "http://sense.puppetme.com/record";
static char* service_uri = "record";
static unsigned int xact_id;
static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

PROCESS(ptemperature_client, "Temperature Sensor & Actuator");

/*******************************************
              Resource Definitions
********************************************/
RESOURCE(stemperature, METHOD_GET, "sensors/temperature");

/*******************************************
            Resource Handlers
********************************************/
static
void read_temperature_sensor(unsigned* temp)
{
  #if PLATFORM_HAS_SHT11
    *temp = -39.60 + 0.01 * temperature_sensor.value(TEMPERATURE_SENSOR);
  #endif
  #ifndef PLATFORM_HAS_SHT11
    *temp = 0; //Platform has no temp sensor at all
  #endif
}

void stemperature_handler(REQUEST* request, RESPONSE* response)
{
  unsigned temperature;

  read_temperature_sensor(&temperature);
  PRINTF("New temperature value %u read.",temperature);
  sprintf(outputBuffer,"%u",temperature);

  //TODO: Enhancement, store e-tag
  rest_set_header_content_type(response, TEXT_PLAIN);
  rest_set_header_payload(response, outputBuffer, strlen(outputBuffer));
}

/*******************************************
            Function Implementation
********************************************/
void ptemperature_initialize()
{
  process_exit(&ptemperature_client);
  //Startup sensor collection.
  SENSORS_ACTIVATE(temperature_sensor);
  rest_activate_resource(&resource_stemperature);
  PRINTF("Temperature sensors initialized successfully");
  //Start client processes next.
  process_start(&ptemperature_client, NULL);
}

static
void send_data()
{
  int data_size = 0;
  clear_buffer(outputBuffer);
  if(init_buffer(COAP_DATA_BUFF_SIZE)){
    coap_packet_t* request =\
    (coap_packet_t*)allocate_buffer(sizeof(coap_packet_t));
    init_packet(request);
    coap_set_method(request, COAP_POST);
    request->tid = xact_id++;
    request->type = MESSAGE_TYPE_CON;
    coap_set_header_uri(request,service_uri);
    coap_set_option(request, Option_Type_Proxy_Uri,
    sizeof(proxy_uri), (uint8_t*)proxy_uri);

    data_size = serialize_packet(request, outputBuffer);

    PRINTF("Now sending request to base station [");
    PRINTF(&client_conn->ripaddr);
    PRINTF("]:%u/%s\n",REMOTE_PORT,service_uri);
    uip_udp_packet_send(client_conn, outputBuffer, data_size);
    delete_buffer();
  }
}

static
void clear_buffer()
{
  memset(outputBuffer,'\x0',sizeof(char)*OUTPUT_BUFFER_SIZE);
}

/********************************************
          Process Definitions
*********************************************/

PROCESS_THREAD(ptemperature_client, ev, data)
{
  static struct etimer atimer;
  PROCESS_BEGIN();
  SERVER_NODE(&server_ipaddr);
  PRINTF("Creating connection to server");
  client_conn = udp_new(&server_ipaddr, UIP_HTONS(REMOTE_PORT), NULL);
  udp_bind(client_conn, UIP_HTONS(LOCAL_PORT));

  PRINTF("Starting ptemperature client timer");
  etimer_set(atimer, CLOCK_SECOND*POLL_INTERVAL);
  while(1)
  {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&atimer));
    PRINTF("Timer expired. Initiating temperature send.");
    send_data();
    etimer_reset(atimer);
  }
  PROCESS_END();
}

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
              Debug Switch
********************************************/
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

char outputBuffer[OUTPUT_BUFFER_SIZE];
static char* proxy_uri = "http://sense.puppetme.com/record";

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
  //Startup sensor collection.
  SENSORS_ACTIVATE(temperature_sensor);
  rest_activate_resource(&resource_stemperature);
  PRINTF("Temperature sensors initialized successfully");
  //Start client processes next.
}

static
void send_data()
{
  clear_buffer(outputBuffer);
  if(init_buffer(COAP_DATA_BUFF_SIZE)){
    coap_packet_t* request =\
    (coap_packet_t*)allocate_buffer(sizeof(coap_packet_t));
    init_packet(request);
    coap_set_method(request, COAP_POST);
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

PROCESS(ptemperature_client, "Temperature Sensor & Actuator");
PROCESS_THREAD(ptemperature_client, ev, data)
{
  //retrieve temperature sensor value
  //construct a COAP response.
  //transmit the response using UDP.
  //wait for timer to expire.
  static struct etimer atimer;
  PROCESS_BEGIN();
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

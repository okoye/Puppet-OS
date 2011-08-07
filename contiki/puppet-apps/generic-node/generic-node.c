/*
 *Licensing information & Copyright info
 *@author: Chuka
 *@email: chuka@puppetme.com
 */

#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "rest.h"

/************************************************
     Include files depending on Target type.
*************************************************/
#if PLATFORM_HAS_SHT && (PLATFORM_HAS_TEMPERATURE || PLATFORM_HAS_HUMIDITY)
#include "dev/sht11-sensor.h"
#endif
#if PLATFORM_HAS_LIGHT
#include "dev/light-sensor.h"
#endif
#if PLATFORM_HAS_LEDS
#include "dev/leds.h"
#endif

/*************************************************
        Debug Switch
**************************************************/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINT_DBG(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

char outputBuffer[OUTPUT_BUFFER_SIZE]; //all outgoing messages copied here.


/**************************************************
              Resource Definitions
***************************************************/
RESOURCE(discover, METHOD_GET,".well-known/core");
RESOURCE(sensors, METHOD_GET,"sensors");
RESOURCE(actuators, METHOD_GET,"actuators");
#if PLATFORM_HAS_TEMPERATURE
RESOURCE(stemperature, METHOD_GET,"sensors/temperature");
#endif
#if PLATFORM_HAS_HUMIDITY
RESOURCE(shumidity, METHOD_GET, "sensors/humidity");
#endif
#if PLATFORM_HAS_LEDS
RESOURCE(aleds, METHOD_POST, "actuators/led");
#endif

/***************************************************
                  Resource Handlers
****************************************************/
void
discover_handler(REQUEST* request, RESPONSE* response)
{
  int index = 0;
  memset(outputBuffer,'',sizeof(char)*OUTPUT_BUFFER_SIZE);
  index += sprintf(outputBuffer+index,"%s,","</sensors>;rt=\"index\"");
  index += sprintf(outputBuffer+index,"%s","</actuators>;rt=\"index\"");

#if PLATFORM_HAS_TEMPERATURE
  index += sprintf(outputBuffer+index,
  "%s,","</sensors/temperature>;rt=\"stemperature\";if=\"sensor\"");
  index += sprintf(outputBuffer+index,
  "%s,","<http://definitions.puppetme.com/sensors/temperature#1>;anchor=\"/sensors/temperature\";rel=\"describedby\"");
#endif
#if PLATFORM_HAS_HUMIDITY
  index += sprintf(outputBuffer+index,
  "%s","</sensors/humidity>;rt=\"shumidity\";if=\"sensor\"");
  index += sprintf(outputBuffer+index,
  "%s,","<http://definitions.puppetme.com/sensors/humidity#1>;anchor=\"/sensors/humidity\";rel=\"describedby\"");
#endif
#if PLATFORM_HAS_LEDS
  index += sprintf(outputBuffer+index,
  "%s","</actuators/led>;rt=\"aleds\";if=\"actuator\"");
  index += sprintf(outputBuffer+index,
  "<http://definitions.puppetme.com/actuators/led#1>;anchor=\"/actuators/led\";rel=\"describedbu\"");
#endif
  rest_set_response_payload(response,(uint8_t*)outputBuffer,strlen(outputBuffer));
  rest_set_header_content_type(response, APPLICATION_LINK_FORMAT);
}

void
sensors_handler(REQUEST* request, RESPONSE* response)
{
  int index = 0;
  memset(outputBuffer, '',sizeof(char)*OUTPUT_BUFFER_SIZE);
  
#if PLATFORM_HAS_TEMPERATURE
  index += sprintf(outputBuffer+index,
  "%s","</sensors/temperature>;rt=\"stemperature\";if=\"sensor\"");
#endif
#if PLATFORM_HAS_HUMIDITY
  index += sprintf(outputBuffer+index,
  "%s","</sensors/humidity>;rt=\"shumidity\";if=\"sensor\"");
#endif
  rest_set_response_payload(response,(uint8_t*)outputBuffer,strlen(outputBuffer));
  rest_set_header_content_type(response, APPLICATION_LINK_FORMAT);
}

void
actuator_handle

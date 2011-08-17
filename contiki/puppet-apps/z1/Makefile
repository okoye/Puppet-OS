CONTIKI=../../

#We require UIP & IPV6 & COAP.
#TODO: Move this to base include file
WITH_UIP=1
UIP_CONF_IPV6=1
WITH_COAP=1

#App specific settings
PLATFORM_HAS_SHT11=1
DEBUG=1

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -DWITH_COAP

APPS += rest-coap
APPS += ptemperature

include $(CONTIKI)/Makefile.include

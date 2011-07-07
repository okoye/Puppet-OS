#ifndef PUPPETOS_H
#define PUPPETOS_H

//Some important constants used in program
enum
{
  SENSOR_SAMPLING_PERIOD = 600,//Frequency of sensor polling
  AM_PUPPET_OS_MSG = 62,//AM Type for Puppet OS messages
  CONFIG_ADDR = 0, //Address to start reading from on disk
  CONFIG_VERSION = 1 //Current version of config_data_t definition.
};

//Data type definitions
enum
{
  true = 1,
  false = 0
};


typedef nx_struct puppet_msg
{
  nx_uint32_t origin_address;
}puppet_msg_t;

typedef struct config_data
{
  char* puppet_service_id; //Assigned puppet service.
  uint32_t node_id; //Assigned randomly by node.
  uint16_t version; //Schema version.
}config_data_t;

#endif

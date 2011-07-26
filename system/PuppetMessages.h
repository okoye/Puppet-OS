#ifndef PUPPETMESSAGES
#define PUPPETMESSAGES

#include "PuppetList.h"
typedef struct sensor_info
{
  nx_char* id;
  nx_char* measurement_unit;
}sensor_info_t;

typedef struct manufacturer_info
{
  nx_char* name;
  nx_char* id;
  nx_char* device_id;
}manufacturer_info_t;

typedef struct register_request
{
  nx_char* device_type;
  nx_char* device_type_id;
  puppet_node_t* sensor_info;
  manufacturer_info_t* m_info;
}register_request_t;

#endif

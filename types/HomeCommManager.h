#ifndef HOME_COMM_MANAGER_H
#define HOME_COMM_MANAGER_H

typedef struct sensor_info
{
  char* id;
  char* measurement_unit;
}sensor_info_t;

enum
{
  BASE = 1,//address of base node.
  AM_REGISTER_REQUEST = 0x10, //Active message ID
};

typedef struct manufacturer_info
{
  char* name;
  char* id;
  char* device_id;
}manufacturer_info_t;

//TODO: Add versioning.
typedef struct register_request
{
  nx_uint32_t device_type_id;
  nx_uint16_t* sensor_ids;
  nx_uint32_t man_id;
}register_request_t;

#endif

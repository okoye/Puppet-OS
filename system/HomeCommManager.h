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
  AM_HOMECOMM = 0x10, //Active message ID
};

typedef struct manufacturer_info
{
  char* name;
  char* id;
  char* device_id;
}manufacturer_info_t;

typedef struct register_request
{
  char* device_type;
  char* device_type_id;
  sensor_info_t* sensor_info;
  manufacturer_info_t* m_info;
}register_request_t;

#endif

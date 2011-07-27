#ifndef PUPPETMESSAGES
#define PUPPETMESSAGES
typedef struct sensor_info
{
  char* id;
  char* measurement_unit;
}sensor_info_t;

enum
{
  BASE = 1,//address of base node.
  HOME_AM_ID = 1,
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

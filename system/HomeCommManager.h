#ifndef HOME_COMM_MANAGER_H
#define HOME_COMM_MANAGER_H

enum
{
  BASE = 1,//address of base node.
  AM_HOMECOMM = 0x10, //Active message ID
};

typedef struct register_request
{
  char* device_type;
  char* device_type_id;
  char** sensor_type_ids;
}register_request_t;

#endif

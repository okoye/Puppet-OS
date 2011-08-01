#ifndef APISERVICE_H
#define APISERVICE_H

enum
{
  SINK_ADDRESS_PREFIX = 0xff02,//address of base node.
  SINK_ADDRESS_SUFFIX = 5,
  SINK_ADDRESS_PORT = 17634,
};

enum
{
  REGISTER_URL = "devices.puppetme.com/register"
}

typedef struct register_request
{
  char* device_type_id;
  char** sensor_ids;
  char* man_id;
}register_request_t;

//TODO: Add signature features.
typedef struct p_message
{
  char* resource_url; //Identifer for URL to post to
  char* http_method; //GET, POST, PUT, DELETE
  uint8_t version; //p_message version
  void* body; //actual data to be jsonified then sent.
}p_message_t;
#endif

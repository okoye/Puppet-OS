#ifndef APISERVICE_H
#define APISERVICE_H

enum
{
  SINK_ADDRESS_PREFIX = 0xff02,//address of base node.
  SINK_ADDRESS_SUFFIX = 5,
  SINK_ADDRESS_PORT = 17634,
};

#define REGISTER_URL "devices.puppetme.com/register" //case 0

typedef struct register_request
{
  nx_uint16_t device_type_id;
  nx_uint16_t* sensor_ids;
  nx_uint32_t man_id;
}register_request_t;

//TODO: Add signature features.
typedef struct p_message
{
  char* resource_url; //Identifer for URL to post to
  char* http_method; //GET, POST, PUT, DELETE
  nx_uint8_t version; //p_message version
  void* body; //actual data to be jsonified then sent.
}p_message_t;

typedef struct p_response
{
  char* resource_url;
  nx_uint16_t http_code;
  void* body;
  uint8_t version;
}p_response_t;
#endif

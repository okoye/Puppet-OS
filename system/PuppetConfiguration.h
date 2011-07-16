#ifndef PUPPETCONFIGURATION
#define PUPPETCONFIGURATION
typedef struct config_data
{
  char* puppet_service_id; //id from puppet service.
  uint16_t node_id; //randomly assigned node id.
  uint16_t version; //configuration version.
}config_data_t;
#endif

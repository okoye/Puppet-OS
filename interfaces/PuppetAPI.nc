#include "PuppetMessages.h"
#include "messages.h"

interface PuppetAPI
{
  command error_t registerDeviceRequest(register_request_t* reg);
  
  event void registerRequestDone(message_t msg, error_t err);

  event void registerDeviceResponse(void* payload);
}

#include "PuppetMessages.h"
#include "message.h"

interface PuppetAPI
{
  command error_t registerDeviceRequest(message_t* reg);
  
  event void registerRequestDone(message_t msg, error_t err);

  event void registerDeviceResponse(void* payload);
}

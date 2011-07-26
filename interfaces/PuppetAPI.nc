#include <PuppetMessages.h>

interface PuppetAPI
{
  command error_t registerDeviceRequest();

  event void registerDeviceResponse(void* payload);
}

#include <APIService.h>
#include "message.h"

interface APIService
{
  /**
   *A register request to the puppet API service containing
   *appropriate messaging parameters including:
   *
   *returns SUCCESS if message was sent successfully without issues
   *        FAIL, EBUSY, ENOMEM or some standard error if there were issues.
   */
  command error_t registerRequest(register_request_t* reg);
 
  /**
   *When a response from the API is received, this event will be fired
   *containing appropriate message and http_code. Upon completion of processing
   *the message_t buffer will be cleared completely therefore its contents
   *must be copied out. Valid HTTP Codes and meanings are maintained.
   */
  event void registerResponse(void* msg, uint16_t http_code);
}

/**
 *@author: Chuka Okoye
 *@email: chuka@puppetme.com
 */

#include <message.h>
#include "HomeCommManager.h"

module HomeCommManagerP
{
  provides
  {
    interface SplitControl;
    interface PuppetAPI;
  }
  uses
  {
    interface SplitControl as RadioControl;
    interface StdControl as RoutingControl;
    interface Send as RadioSend;
    interface Leds;
  }
}
implementation
{
  //method definitions.
  error_t validateRegisterRequest(register_request_t* reg);
  error_t sendPacket(register_request_t* req);

  //global state variables.
  int ready = 0;
  message_t message_buf;

  command error_t SplitControl.start()
  {
    //start radio, initialize api components
    return call RadioControl.start();
  }

  event void RadioControl.startDone(error_t err)
  {
    //signal completion.
    if (err == SUCCESS)
      ready = 1;
    signal SplitControl.startDone(err);
  }

  command error_t SplitControl.stop()
  {
    //stop radio and deallocate resources
    return call RadioControl.stop();
  }

  event void RadioControl.stopDone(error_t err)
  {
    if (err == SUCCESS)
      ready = 0;
    signal SplitControl.stopDone(err);
  }
  
  command error_t PuppetAPI.registerDeviceRequest(register_request_t* req)
  {
    if (req == NULL)
      return FAIL;
    //validate register elements
    if (validateRegisterRequest(req) != SUCCESS)
      return FAIL;
    else
    {
      //prepare to send info if already initialized.
      error_t err = sendPacket(req);
      return err; //TODO: Log err for instrumentation.
    }
  }
  
  event void RadioSend.sendDone(message_t* msg, error_t e)
  {
    if(e == SUCCESS)
      signal PuppetAPI.registerRequestDone(msg, e);
    else
    {
      //TODO: Log data for instrumentation
      signal PuppetAPI.registerRequestDone(msg, e);
    }
  }

  error_t sendPacket(register_request_t* req)
  {
    error_t err = FAIL;
    if(ready)
    {
      register_request_t* ptr =(register_request_t*) call RadioSend.getPayload(&message_buf,sizeof(register_request_t));
      memcpy(ptr,req,sizeof(register_request_t));
      err = call RadioSend.send(&message_buf,sizeof(register_request_t));
      call Leds.led2On();
    }
    return FAIL;
  }

  error_t validateRegisterRequest(register_request_t* reg)
  {
    if (reg->device_type != NULL &&
        reg->device_type_id != NULL &&
        reg->sensor_type_ids != NULL)
        return SUCCESS;
    else
      return FAIL;
  }
}

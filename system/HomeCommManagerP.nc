/**
 *@author: Chuka Okoye
 *@email: chuka@puppetme.com
 */

#include <AM.h>
#include <message.h>
#include "PuppetMessages.h"

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
  error_t sendPacket(message_t* msg, int size);

  //global state variables.
  int ready = 0;

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
  
  command error_t PuppetAPI.registerDeviceRequest(message_t* msg)
  {
    register_request_t* reg = call Packet.getPayload(msg,sizeof(register_request_t));
    if (reg == NULL)
      return FAIL;
    //validate register elements
    if (validateRegisterRequest(reg) != SUCCESS)
      return FAIL;
    else
    {
      //prepare to send info if already initialized.
      error_t err = sendPacket(msg,sizeof(register_request_t));
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

  error_t sendPacket(message_t* msg, size)
  {
    error_t err = FAIL;
    if(ready)
      err = call RadioSend.send(msg,size);
    call Leds.led2On();
    return err;
  }

  error_t validateRegisterRequest(register_request_t* reg)
  {
    if (reg->device_type != NULL &&
        reg->device_type_id != NULL &&
        reg->sensor_info->id != NULL &&
        reg->sensor_info->measurement_unit != NULL &&
        reg->m_info != NULL)
        return SUCCESS;
    else
      return FAIL;
  }
}

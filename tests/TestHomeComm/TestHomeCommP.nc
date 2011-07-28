#include "TestCase.h"
#include "HomeCommManager.h"

module TestHomeCommP
{
  uses
  {
    interface TestCase as TestSplitControl;
    interface TestCase as TestAPIRegisterDevice;
    interface SplitControl as HomeCommControl;
    interface PuppetAPI as API;
    interface Packet;
  }
}
implementation
{
  register_request_t* reg; 

  event void TestSplitControl.run()
  {
    error_t value = call HomeCommControl.start();
    assertTrue("Could not start HomeCommControl",SUCCESS==value);
    if (value != SUCCESS)
      call TestSplitControl.done(); //prevent from timing out.
  }
  event void HomeCommControl.startDone(error_t value)
  {
    assertEquals("HomeComm start failed",SUCCESS,value);
    call HomeCommControl.stop();
  }
  
  event void HomeCommControl.stopDone(error_t value)
  {
    assertEquals("HomeComm stop failed",SUCCESS,value);
    call TestSplitControl.done();
  }

  event void TestAPIRegisterDevice.run()
  {
    error_t err1, err2;
    reg = (register_request_t*)malloc(sizeof(register_request_t));
    
    //initialize struct
    reg->device_type = "FRIDGE";
    reg->device_type_id = "01";
    reg->sensor_type_ids[0] = "3452";
    reg->manufacturer_name = "Samsung";
    reg->manufacturer_id = "034567";

    call HomeCommControl.start(); //re-initialize api.
    err1 = call API.registerDeviceRequest(reg);
    err2 = call API.registerDeviceRequest(NULL);
    assertTrue("Failed to register device",err1 == TRUE);
    assertEquals("Register device should fail", err2,FAIL);
    assertResultIsBelow("Message greater than payload size",
          sizeof(register_request_t),call Packet.maxPayloadLength());
    if (err1 == FAIL || err2 == SUCCESS)
      call TestAPIRegisterDevice.done(); //prevent from timing out
    //TODO: More negative tests for validation.
  }

  event void API.registerRequestDone(message_t* msg, error_t e)
  {
    assertTrue("Failed to send message",e==SUCCESS);
    call TestAPIRegisterDevice.done();
    assertEquals("Failed to stop HomeComm",
          call HomeCommControl.stop(),SUCCESS);
  }

  event void API.registerDeviceResponse(void* res)
  {}

}

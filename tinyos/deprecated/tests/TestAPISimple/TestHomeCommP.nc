#include "TestCase.h"
#include "HomeCommManager.h"

module TestHomeCommP
{
  uses
  {
    interface TestControl as Setup;
    interface TestControl as TearDown;
    interface TestCase as TestAPIRegisterDevice;
    interface SplitControl as HomeCommControl;
    interface PuppetAPI as API;
    interface Packet;
  }
}
implementation
{
  register_request_t* reg; 

  event void Setup.run()
  {
    assertTrue("Could not start HomeCommControl",
      call HomeCommControl.start()==SUCCESS);
  }

  event void TearDown.run()
  {
    assertTrue("Failed to stop HomeCommControl",
            call HomeCommControl.stop()==SUCCESS);
  }

  event void HomeCommControl.startDone(error_t value)
  {
    assertEquals("HomeComm start failed",SUCCESS,value);
    call Setup.done();
  }
  
  event void HomeCommControl.stopDone(error_t value)
  {
    assertEquals("HomeComm stop failed",SUCCESS,value);
    call TearDown.done();
  }

  event void TestAPIRegisterDevice.run()
  {
    assertSuccess();
    reg = (register_request_t*)malloc(sizeof(register_request_t));
    
    reg->device_type_id = 01;
    reg->sensor_ids[0] = 5678;
    reg->man_id = 345678;

    assertTrue("Register device should fail",
      call API.registerDeviceRequest(NULL)==FAIL);
    assertResultIsBelow("Register Message greater than Packet",
          call Packet.maxPayloadLength(),sizeof(register_request_t));
    
    call TestAPIRegisterDevice.done(); //prevent from timing out
  }

  event void API.registerRequestDone(message_t* msg, error_t e)
  {}

  event void API.registerDeviceResponse(void* res)
  {}

}

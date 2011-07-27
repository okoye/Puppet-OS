#include "TestCase.h"
#include "PuppetMessages.h"
#include "message.h"

module TestHomeCommP
{
  uses
  {
    interface TestControl as SetUp;
    interface TestCase as TestSplitControl;
    interface TestCase as TestAPIRegisterDevice;
    interface SplitControl as HomeCommControl;
    interface PuppetAPI as API;
    interface Packet;
  }
}
implementation
{
  message_t msg;
  register_request_t* reg; 

  event void SetUp.run()
  {
    //retrieve pointer to payload
    reg = call Packet.getPayload(&msg,sizeof(register_request_t));
    assertTrue("Message bigger than packet allows", reg != NULL); //message should not be bigger than packet allows
    //now setup struct with data.
    reg->device_type = "FRIDGE";
    reg->device_type_id = "01";
    reg->sensor_info->id = "3452";
    reg->sensor_info->measurement_unit = "Farenheit";
    reg->m_info->name = "Samsung";
    reg->m_info->id = "034567";
  }
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
  }
  
  event void HomeCommControl.stopDone(error_t value)
  {
    assertEquals("HomeComm stop failed",SUCCESS,value);
    assertEquals("Register device should fail",\
      call API.registerDeviceRequest(NULL),FAIL); //fails because of null msg
    //TODO: Add more negative tests for validation.
    assertEquals("Reg"
    call TestSplitControl.done();
  }

  event void TestAPIRegisterDevice.run()
  {
    assertTrue("Failed to register device",
          call API.registerDeviceRequest(msg) == SUCCESS);
  }

  event void API.registerRequestDone(message_t* msg, error_t e)
  {
    assertTrue("Failed to send message",e==SUCCESS);
    TestAPIRegisterDevice.done();
    assertEquals("Failed to stop HomeComm",
          call HomeCommControl.stop(),SUCCESS);
  }

  event void registerDeviceResponse(message_t* msg)
  {
  }

}

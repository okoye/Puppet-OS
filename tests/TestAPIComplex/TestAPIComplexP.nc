#include "TestCase.h"
#include "HomeCommManager.h"

module TestAPIComplexP
{
  uses
  {
    interface TestControl as SetUpOneTime;
    interface TestControl as TearDownOneTime;
    interface TestCase as TestRegister;
    interface SplitControl;
    interface PuppetAPI;
    interface RootControl;
    interface Receive;
  }
}
implementation
{
  register_request_t* reg;

  /**Run once by all nodes before test start**/
  event void SetUpOneTime.run()
  {
    call SplitControl.start();
    //Check if node id is 1, if so, set root.
    if(TOS_NODE_ID)
    {
      call RootControl.setRoot();
    }
  }
  /**Run once by all nodes after all tests completed**/
  event void TearDownOneTime.run()
  {
    call SplitControl.stop();
  }

  event void SplitControl.startDone(error_t err)
  {
    call SetUpOneTime.done();
  }

  event void SplitControl.stopDone(error_t err)
  {
    call TearDownOneTime.done();
  }

  event void TestRegister.run()
  {
    reg = (register_request_t*)malloc(sizeof(register_request_t));

    reg->device_type = "FRIDGE";
    reg->device_type_id = "00";
    reg->sensor_info->id = "345678";
    reg->sensor_info->measurement_unit = "Watts";
    reg->m_info->id = "4567989";
    reg->m_info->name = "Samsung";

    assertTrue("Could not send message",
      call PuppetAPI.registerDeviceRequest(reg)==SUCCESS);
  }

  event void PuppetAPI.registerRequestDone(message_t* msg, error_t err)
  {
    assertTrue("Message was not sent",err==SUCCESS);
  }

  event void PuppetAPI.registerDeviceResponse(void* res)
  {
    //Should do nothing
  }

  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len)
  {
    assertSuccess();
    call TestRegister.done();
  }
}

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
    interface Receive as Snoop;
    interface Receive;
    interface Leds;
  }
}
implementation
{
  //TODO: Add more negative tests
  register_request_t* reg;

  enum{
    NUM_OF_PACKETS = 1000,
  };

  /**Run once by all nodes before test start**/
  event void SetUpOneTime.run()
  {
    call SplitControl.start();
    call Leds.led2On();
    call RootControl.setRoot(); //TODO: Make use of Collection Snoop debugging
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

    reg->device_type_id = 1;
    reg->sensor_ids[0] = 30;
    reg->man_id = 1678902;

    assertTrue("Could not send message",
      call PuppetAPI.registerDeviceRequest(reg)==SUCCESS);
  }

  event void PuppetAPI.registerRequestDone(message_t* msg, error_t err)
  {
    assertTrue("Message was not sent",err==SUCCESS);
    assertTrue("Message was NULL", msg != NULL);
  }

  event void PuppetAPI.registerDeviceResponse(void* res)
  {
    //Do nothing
  }

  event message_t* Snoop.receive(message_t* msg, void* payload, uint8_t len)
  {
    assertNotNull(msg);
    return msg;
  }

  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len)
  {
    assertNotNull(msg);
    call TestRegister.done();
    return msg;
  }

}

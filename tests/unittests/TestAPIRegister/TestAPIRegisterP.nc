#include "TestCase.h"
#include <APIService.h>

module TestAPIRegisterP{
  uses{
    interface TestControl as SetUpOneTime;
    interface TestControl as TearDownOneTime;
    interface TestCase as TestRegister;
    interface SplitControl;
    interface APIService;
  }
}
implementation{
  event void SetUpOneTime.run(){
    call SplitControl.start();
  }
  event void TearDownOneTime.run(){
    call SplitControl.stop();
  }
  event void SplitControl.startDone(error_t err){
    call SetUpOneTime.done();
  }
  event void SplitControl.stopDone(error_t err){
    call TearDownOneTime.done();
  }
  event void TestRegister.run(){
    register_request_t reg;
    error_t err;
    reg.device_type_id = 1;
    reg->sensor_ids[0] = 1;
    reg->man_id = 1;

    err = call APIService.registerRequest(reg);
    assertTrue("Register request failed to send",err==SUCCESS);
    call TestRegister.done();
  }
}

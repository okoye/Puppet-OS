#include <APIService.h>
#include "printf.h"

module TestAPIRegisterP{
  uses{
    interface Boot;
    interface SplitControl;
    interface APIService;
    interface Timer<TMilli>;
  }
}
implementation{
  void testRegister();
  void logError(char* err);
  uint16_t count;

  event void Boot.booted(){
    call SplitControl.start();
    call Timer.startPeriodic(300);
    logError("Booted up successfully");
  }
  event void Timer.fired(){
    printf("Timer fired.\n");
    printfflush();
    testRegister();
  }
  event void SplitControl.startDone(error_t err){
    logError("Radio started successfully");
  }
  event void SplitControl.stopDone(error_t err){
  }
  void testRegister(){
    register_request_t* reg;
    error_t err;
      
    count++;
    printf("Allocating memory for register_request\n");
    reg = (register_request_t*)malloc(sizeof(register_request_t));
    printf("Allocated memory for register\n");
    reg->device_type_id = 1;
    reg->sensor_ids[0] = 1;
    reg->man_id = 1;
    printf("Initialized register data structure\n");
    err = call APIService.registerRequest(reg);
    printf("Result: %d\tCount: %d\n",err,count);
    free(reg);
    printf("Cleaning up allocated memory\n");
    printfflush();
  }
  void logError(char* err){
    printf(err);
    printf("\n");
    printfflush();
  }
  event void APIService.registerResponse(void* msg, uint16_t http_code){
  }
}

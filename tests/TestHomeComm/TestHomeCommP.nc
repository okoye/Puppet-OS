#include "TestCase.h"

module TestHomeCommP
{
  uses
  {
    interface TestCase as TestSplitControl;
    interface SplitControl as HomeCommControl;
  }
}
implementation
{
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
    value = call HomeCommControl.stop();
    assertTrue("Could not stop HomeCommControl", SUCCESS==value);
  }
  
  event void HomeCommControl.stopDone(error_t value)
  {
    assertEquals("HomeComm stop failed",SUCCESS,value);
    call TestSplitControl.done();
  }
}

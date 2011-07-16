#include "TestCase.h"

module FirstTestP
{
  uses interface TestCase as BasicAssertionTest;
}
implementation
{
  event void BasicAssertionTest.run()
  {
    assertTrue("Not false",TRUE);
    assertFalse("Not true",FALSE);
    assertNotEquals("Should not be equal",(float)1.2,(int)1.2);
    call BasicAssertionTest.done();
  }
}

#include "HomeCommManager.h"

configuration TestAPIComplexC
{}
implementation
{
  components new TestCaseC() as TestRegisterC;
  components new StatisticsC() as Stats;
  components HomeCommManagerC;
  components CollectionC;
  components TestAPIComplexP;
  components LedsC;
 
  TestAPIComplexP.SetUpOneTime -> TestRegisterC.SetUpOneTime;
  TestAPIComplexP.TearDownOneTime -> TestRegisterC.TearDownOneTime;
  TestAPIComplexP.TestRegister -> TestRegisterC;
  TestAPIComplexP.SplitControl -> HomeCommManagerC;
  TestAPIComplexP.PuppetAPI -> HomeCommManagerC;
  TestAPIComplexP.RootControl -> CollectionC;
  TestAPIComplexP.Receive -> CollectionC.Receive[AM_REGISTER_REQUEST];
  TestAPIComplexP.Snoop -> CollectionC.Snoop[AM_REGISTER_REQUEST];
  TestAPIComplexP.Leds -> LedsC;
}

#include "HomeCommManager.h"

configuration TestAPIComplexC
{}
implementation
{
  components new TestCaseC() as TestRegisterC;
  components HomeCommManagerC;
  components CollectionC;
  components TestAPIComplexP;

  TestAPIComplexP.SetUpOneTime -> TestRegisterC.SetUpOneTime;
  TestAPIComplexP.TearDownOneTime -> TestRegisterC.TearDownOneTime;
  TestAPIComplexP.TestRegister -> TestRegisterC;
  TestAPIComplexP.SplitControl -> HomeCommManagerC;
  TestAPIComplexP.PuppetAPI -> HomeCommManagerC;
  TestAPIComplexP.RootControl -> CollectionC;
  TestAPIComplexP.Receive -> CollectionC.Receive[AM_HOMECOMM];
}

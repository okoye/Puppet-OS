configuration TestHomeCommC
{}
implementation
{
  components new TestCaseC() as TestSplitControlC,
            new TestCaseC() as TestAPIRegisterDeviceC,
            new TestCaseC() as TestControl,
            HomeCommManagerC as HomeController,
            TestHomeCommP;

  TestHomeCommP.TestSplitControl -> TestSplitControlC;
  TestHomeCommP.TestAPIRegisterDevice -> TestAPIRegisterDeviceC;
  TestHomeCommP.HomeCommControl -> HomeController;
  TestHomeCommP.SetUp -> TestControl;
  TestHomeCommP.API -> HomeController;
}

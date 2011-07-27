configuration TestHomeCommC
{}
implementation
{
  components new TestCaseC() as TestSplitControlC;
  components new TestCaseC() as TestAPIRegisterDeviceC;
  components new TestCaseC() as TestControl;
  components HomeCommManagerC as HomeController;
  components TestHomeCommP;
  components DymoNetworkC;

  TestHomeCommP.TestSplitControl -> TestSplitControlC;
  TestHomeCommP.TestAPIRegisterDevice -> TestAPIRegisterDeviceC;
  TestHomeCommP.HomeCommControl -> HomeController;
  TestHomeCommP.SetUp -> TestControl;
  TestHomeCommP.API -> HomeController;
  TestHomeCommP.Packet -> DymoNetworkC;
}

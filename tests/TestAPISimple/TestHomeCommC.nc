configuration TestHomeCommC
{}
implementation
{
  components new TestCaseC() as TestSplitControlC,
            new TestCaseC() as TestAPIRegisterDeviceC,
            HomeCommManagerC as HomeController,
            CollectionC,
            TestHomeCommP;

  TestHomeCommP.TestSplitControl -> TestSplitControlC;
  TestHomeCommP.TestAPIRegisterDevice -> TestAPIRegisterDeviceC;
  TestHomeCommP.HomeCommControl -> HomeController;
  TestHomeCommP.API -> HomeController;
  TestHomeCommP.Packet -> CollectionC.Packet;
}

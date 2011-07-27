configuration TestHomeCommC
{}
implementation
{
  components new TestCaseC() as TestSplitControlC,
            new CollectionSender(0x13),
            TestCaseC() as TestAPIRegisterDeviceC,
            TestCaseC() as TestControl,
            HomeCommManagerC as HomeController,
            TestHomeCommP;

  TestHomeCommP.TestSplitControl -> TestSplitControlC;
  TestHomeCommP.TestAPIRegisterDevice -> TestAPIRegisterDeviceC;
  TestHomeCommP.HomeCommControl -> HomeController;
  TestHomeCommP.SetUp -> TestControl;
  TestHomeCommP.API -> HomeController;
  TestHomeCommP.Send -> CollectionSender;
}

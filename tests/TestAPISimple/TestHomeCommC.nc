configuration TestHomeCommC
{}
implementation
{
  components new TestCaseC() as TestAPIRegisterDeviceC,
            HomeCommManagerC,
            CollectionC,
            TestHomeCommP;

  TestHomeCommP.Setup -> TestAPIRegisterDeviceC.SetUpOneTime;
  TestHomeCommP.TearDown -> TestAPIRegisterDeviceC.TearDownOneTime;
  TestHomeCommP.TestAPIRegisterDevice -> TestAPIRegisterDeviceC;
  TestHomeCommP.HomeCommControl -> HomeCommManagerC;
  TestHomeCommP.API -> HomeCommManagerC;
  TestHomeCommP.Packet -> CollectionC.Packet;
}

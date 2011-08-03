configuration TestAPIRegisterC{}
implementation{
  components new TestCaseC() as TestRegisterC, APIServiceC, TestAPIRegisterP;
  components LedsC;

  TestAPIRegisterP.SetUpOneTime -> TestRegisterC.SetUpOneTime;
  TestAPIRegisterP.TearDownOneTime -> TestRegisterC.TearDownOneTime;
  TestAPIRegisterP.TestRegister -> TestRegisterC;
  TestAPIRegisterP.SplitControl -> APIServiceC;
  TestAPIRegisterP.APIService -> APIServiceC;
  TestAPIRegisterP.Leds -> LedsC;
}

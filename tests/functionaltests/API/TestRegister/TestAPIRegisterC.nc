configuration TestAPIRegisterC{}
implementation{
  components  APIServiceC, TestAPIRegisterP;
  components new TimerMilliC() as Timer0, MainC;

  TestAPIRegisterP.SplitControl -> APIServiceC;
  TestAPIRegisterP.APIService -> APIServiceC;
  TestAPIRegisterP.Timer -> Timer0;
  TestAPIRegisterP.Boot -> MainC.Boot;
}

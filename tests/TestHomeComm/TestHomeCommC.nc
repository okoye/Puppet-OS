configuration TestHomeCommC
{}
implementation
{
  components new TestCaseC() as TestSplitControlC;
  components HomeCommManagerC as HomeController;
  components TestHomeCommP;

  TestHomeCommP.TestSplitControl -> TestSplitControlC;
  TestHomeCommP.HomeCommControl -> HomeController;
}

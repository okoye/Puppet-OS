configuration FirstTestC
{}
implementation
{
  components new TestCaseC() as BasicAssertionTestC;
  components FirstTestP;

  FirstTestP.BasicAssertionTest -> BasicAssertionTestC;
}

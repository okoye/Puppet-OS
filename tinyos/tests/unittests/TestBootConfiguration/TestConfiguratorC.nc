configuration TestConfiguratorC
{}
implementation
{
  components new TestCaseC() as TestReadConfigC;
  components new TestCaseC() as TestWriteConfigC;
  components BootConfiguratorC;
  components TestConfiguratorP;

  TestConfiguratorP.TestReadConfig -> TestReadConfigC;
  TestConfiguratorP.TestWriteConfig -> TestWriteConfigC;
  TestConfiguratorP.BootConfigurator -> BootConfiguratorC;
}

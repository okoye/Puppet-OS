#include "TestCase.h"
#include "PuppetConfiguration.h"

module TestConfiguratorP
{
  uses
  {
    interface TestCase as TestReadConfig;
    interface TestCase as TestWriteConfig;
    interface PuppetConfigurator;
  }
}
implementation
{
  event void TestWriteConfig.run()
  {
    call PuppetConfigurator.writeConfig(NULL);
  }
  event void TestReadConfig.run()
  {
    call PuppetConfigurator.configure();
  }
  event void PuppetConfigurator.configureDone(error_t err, config_data_t* c)
  {
    assertEquals("err was FAIL", SUCCESS, err);
    assertNull(c);
    call TestReadConfig.done();
  }
  event void PuppetConfigurator.writeConfigDone(error_t err)
  {
    assertEquals("failed to write", SUCCESS, err);
    call TestWriteConfig.done();
  }
}

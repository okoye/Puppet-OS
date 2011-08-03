#include "TestCase.h"
#include "BootConfigurator.h"

module TestConfiguratorP
{
  uses
  {
    interface TestCase as TestReadConfig;
    interface TestCase as TestWriteConfig;
    interface BootConfigurator;
  }
}
implementation
{
  event void TestWriteConfig.run()
  {
    call BootConfigurator.writeConfig(NULL);
  }
  event void TestReadConfig.run()
  {
    call BootConfigurator.configure();
  }
  event void BootConfigurator.configureDone(error_t err, config_data_t* c)
  {
    assertTrue("err was FAIL", SUCCESS == err);
    assertNull(c);
    call TestReadConfig.done();
  }
  event void BootConfigurator.writeConfigDone(error_t err)
  {
    assertTrue("failed to write", SUCCESS==err);
    call TestWriteConfig.done();
  }
}

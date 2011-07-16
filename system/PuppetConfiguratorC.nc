/**
 *@author: Chuka Okoye
 *@email: chuka@puppetme.com
 */
#include "StorageVolumes.h"

configuration PuppetConfiguratorC
{
  provides
  {
    interface PuppetConfigurator;
  }
  
}

implementation
{
  components new ConfigStorageC(VOLUME_CONFIGTEST);
  components PuppetConfiguratorP;

  PuppetConfigurator = PuppetConfiguratorP;

  PuppetConfiguratorP.Config -> ConfigStorageC;
  PuppetConfiguratorP.Mount -> ConfigStorageC;
}

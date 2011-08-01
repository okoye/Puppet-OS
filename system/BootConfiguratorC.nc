/**
 *@author: Chuka Okoye
 *@email: chuka@puppetme.com
 */
#include "StorageVolumes.h"

configuration BootConfiguratorC
{
  provides
  {
    interface BootConfigurator;
  }
  
}

implementation
{
  components new ConfigStorageC(VOLUME_CONFIGTEST);
  components BootConfiguratorP;

  BootConfigurator = BootConfiguratorP;

  BootConfiguratorP.Config -> ConfigStorageC;
  BootConfiguratorP.Mount -> ConfigStorageC;
}

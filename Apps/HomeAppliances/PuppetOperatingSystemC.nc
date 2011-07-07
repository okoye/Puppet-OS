/*
 *Copyright info here
 */

/*
 *@author: Chuka Okoye
 */
#include "StorageVolumes.h"
configuration PuppetOperatingSystemC
{
}
implementation
{
  components MainC, LedsC, ActiveMessageC;
  components PuppetOperatingSystemP as Puppet;
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components new ConfigStorageC(VOLUME_CONFIGTEST);

  Puppet.Boot -> MainC;
  Puppet.Leds -> LedsC;
  Puppet.SenseTimer -> Timer0;
  Puppet.PuppetDatastoreTimer -> Timer1;
  Puppet.RadioControl -> ActiveMessageC;
  Puppet.Mount -> ConfigStorageC;
  Puppet.Config -> ConfigStorageC;
}

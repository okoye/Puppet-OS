/**
 *@author: Chuka Okoye
 *@email: chuka@puppetme.com
 */

#include "AM.h"

module HomeCommManagerP
{
  provides
  {
    interface SplitControl;
    interface PuppetAPI;
  }
  uses
  {
    interface SplitControl as RadioControl;
    interface AMSend as RadioSend[am_id_t id];
    interface Receive as RadioReceive[am_id_t id];
  }
}
implementation
{
  command error_t SplitControl.start()
  {
    //start radio, initialize api components
    return call RadioControl.start();
  }

  event void RadioControl.startDone(error_t err)
  {
    //signal completion.
    signal SplitControl.startDone(err);
  }

  command error_t SplitControl.stop()
  {
    //stop radio and deallocate resources
    return call RadioControl.stop();
  }

  event void RadioControl.stopDone(error_t err)
  {
    signal SplitControl.stopDone(err);
  }
  
  command error_t PuppetAPI.registerDeviceRequest(register_t reg)
  {
    //validate register elements

  }

  
}

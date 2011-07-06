/*
 * Copyright info here
 */

/*
 * @author Chuka Okoye
 * Revision: $Id: PuppetOperatingSystemP.nc, v0.1 2011/07/5 19:34:04 chuka Exp
 */

/*
 * PuppetOperatingSystemP senses available sensors, encodes it in the 
 * appropriate format (JSON) and sends it data to appropriate datastore
 */

module PuppetOperatingSystemP
{
  uses
  {
    interface Boot;
    interface Read<uint16_t>; //TODO: Hide this interface
    interface Timer<TMilli>;
    interface Leds;
  }
}
implementation
{
  event void Boot.booted()
  {
    //TODO: perform initialization of hardware & software.

  }
}

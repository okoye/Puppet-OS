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

#include "PuppetOperatingSystem.h"
#include "StorageVolumes.h"

module PuppetOperatingSystemP
{
  uses
  {
    interface Boot;
    interface Read<uint16_t>; //TODO: Hide this interface
    interface Timer<TMilli> as SenseTimer; //For sensors needing polling
    interface Timer<TMilli> as PuppetDatastoreTimer;//Update cloud datastore
    interface SplitControl as RadioControl;
    interface ConfigStorage;
    interface Mount;
    interface Leds;
  }
}
implementation
{
  void initialize_hardware();//setup timer, radio elements
  void initialize_software();
  void debug_leds(int status);
  bool read_config(config_data_t* info); //reads config data from storage.

  
  bool ready = false; //set whenever all initialization is complete.
  
  /******************************************
            Events Definition
  ******************************************/
  event void Boot.booted()
  {
    //TODO: perform initialization of hardware & software.
    initialize_hardware();
    initialize_software();
  }

  /*******************************************
            C Function Definitions
  ********************************************/

  void initialize_hardware()
  {
    /*
    Initialize flash drive, timer, sensors and 
    */
    post 
  }

  void initialize_software()
  {
    /*
    Platform should check that it has received a unique identification
    from cloud platform. This ID is used to identify, manage and communicate
    with the sensing platform.
    */
    config_data_t* my_config;

    //First, we want to read config information from log
    
    read_config(&my_config);

  }

  bool read_config(config_data_t* info)
  {
    /*
    Reads config information from storage and stores
    in config_data_t object.
    */
    
  }

}

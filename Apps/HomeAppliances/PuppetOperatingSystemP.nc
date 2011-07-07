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
    interface ConfigStorage as Config;
    interface Mount;
    interface Leds;
  }
}
implementation
{
  void initializeHardware();//setup timer, radio elements
  void initializeSoftware();
  void debugLeds(int status);
  bool readConfig(config_data_t* info); //reads config data from storage.

  
  bool ready = 0; //set whenever all initialization is complete.
  bool mount_completed = 0; //set when drives are successfully mounted
  config_data_t* my_config = (config_data_t*)malloc(sizeof(config_data_t));

  /******************************************
            Events Implementation
  ******************************************/
  event void Boot.booted()
  {
    //TODO: perform initialization of hardware & software.
    initialize_hardware();
    initialize_software();
  }

  event void Mount.mountDone(error_t error)
  {
     atomic
     {
      if (error == SUCCESS)
      {
        if(call Config.valid() == TRUE)//Has some config data on it.
        {
          if(call Config.read(CONFIG_ADDR,
                    my_config,sizeof(config_data_t)) != SUCCESS)
          {
            
          }
        }
        else //Need to commit some data
        {
          
        }
      }
      else
      {
        post mountDrives();
      }
     }
  }

  /*******************************************
              Task Definitions
  ********************************************/
  task void mountDrives()
  {
    if (call Mount.mount() != SUCCESS)
    {
      post mountDrives();
    }
  }
  /*******************************************
            C Function Implementation
  ********************************************/

  void initializeHardware()
  {
    /*
    Initialize flash drive, timer, sensors and 
    */
    post mountDrives();
  }

  void initializeSoftware()
  {
    /*
    Platform should check that it has received a unique identification
    from cloud platform. This ID is used to identify, manage and communicate
    with the sensing platform.
    */
  }

  bool readConfig(config_data_t* info)
  {
    /*
    Reads config information from storage and stores
    in config_data_t object.
    */
    
  }
}

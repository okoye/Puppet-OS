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

module PuppetOperatingSystemP
{
  uses
  {
    interface Boot;
    //interface Read<uint16_t>; //TODO: Hide this interface
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
  bool configureSystem(config_data_t* info); //reads config data from storage.

  
  bool ready = 0; //set whenever all initialization is complete.
  bool mount_completed = 0; //set when drives are successfully mounted

  config_data_t my_config;

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
    if (error == SUCCESS)
    {
      mount_completed = 1;
      if(call Config.valid() == TRUE)//Has some config data on it.
      {
        post readConfigInfo();
      }
      else //Need to commit some data
      {
        configureSystem();
      }
    }
    else
    {
        //TODO: Fatal Error. Log Error.
    }
  }

  event void Config.readDone(storage_addr_t addr, void* buf,
    storage_len_t len, error_t err)__attribute__((noinline))
  {
    if(err == SUCCESS)
    {
      memcpy(&my_config,buf,len);
      if (my_config.version != CONFIG_VERSION)
      {
        //TODO: Warn with LEDS.
      }
      post writeConfigInfo();
    }
  }

  event void Config.writeDone(storage_addr_t addr, void *buf,
    storage_len_t len, error_t err)
  {
    if(err == SUCCESS)
    {
      post commitConfigInfo();
    }
    else
    {
      post writeConfigInfo(); //TODO: Log info.
    }
  }

  event void Config.commitDone(error_t error)
  {
    if(error != SUCCESS)
      post commitConfigInfo(); //TODO: Log info
  }

  /*******************************************
              Task Definitions
  ********************************************/
  task void mountDrives()
  {
    if (call Mount.mount() != SUCCESS)
    {
      //TODO: Fatal Error. Log Error.
    }
  }

  task void readConfigInfo()
  {
    error_t result = Config.read(CONFIG_ADDR,&my_config,sizeof(my_config));
    if (result != SUCCESS)
    {
      if (result == EBUSY)
      {
        post readConfigInfo();
      }
      else
      {
        //TODO: Log error to LEDS.
      }
    }
  }

  task void writeConfigInfo()
  {
    call Config.write(CONFIG_ADDR, &my_config, sizeof(my_config));
  }

  task void commitConfigInfo()
  {
    if (Config.commit() != SUCCESS)
      post commitConfigInfo();
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

  bool configureSystem(config_data_t* info)
  {
    /*
    Configure system information
    */
    //First, generate unique node id.
    info->node_id =

    //Next, encode requests using HTTP Framework & Avro.
  }
}

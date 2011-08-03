/**
 *@author: Chuka Okoye
 *@email: chuka@puppetme.com
 */

#include "BootConfigurator.h"
#include <stdlib.h>
//TODO: Add support for state interface.

module BootConfiguratorP
{
  provides interface BootConfigurator;
  uses
  {
    interface ConfigStorage as Config;
    interface Mount;
  }
}
implementation
{
  void signalReadCompletion(error_t err, config_data_t*);
  void signalWriteCompletion(error_t err);

  config_data_t* config = NULL;

  command error_t BootConfigurator.configure()
  {
    
    config = (config_data_t*)malloc(sizeof(config_data_t));
    if(!config)
      return ENOMEM;
    return call Mount.mount();//fail if it has already been mounted.
  }

  event void Mount.mountDone(error_t err)
  {
    if (err == FAIL)
    {
      //some major internal problem
      signalReadCompletion(FAIL, NULL);
    }
    else if(err == SUCCESS)
    {
      //now go to next state, READ
      if(call Config.valid() == TRUE)
      {
        //read config information
        error_t result = call Config.read(CONFIG_ADDRESS,
                                            config,
                                            sizeof(config));
        if(result == EBUSY)
          signalReadCompletion(EBUSY,NULL);
        else if(result == SUCCESS)
        {
          //do nothing, wait for event callback.
        }
        else if(result == FAIL)
        {
          //does not contain valid data for some reason.
          signalReadCompletion(SUCCESS,NULL);
        }
        else
        {
          signalReadCompletion(FAIL,NULL);
        }
      }
      else
      {
        //probably no data on config partition yet.
        signalReadCompletion(SUCCESS, NULL);
      }
    }
  }

  event void Config.readDone(storage_addr_t add, void* buf,
    storage_len_t len, error_t err)__attribute__((noinline))
  {
    if(err == SUCCESS)
    {
      //go to final state FINISHED
      memcpy(config,buf,len);
      signalReadCompletion(SUCCESS, config);
    }
    else
    {
      signalReadCompletion(FAIL, NULL);
    }
  }

  command void BootConfigurator.writeConfig(config_data_t* data)
  {
    //just to be safe, remount incase it is not mounted
    call Mount.mount();
    call Config.write(CONFIG_ADDRESS, data, sizeof(*data));
  }

  event void Config.writeDone(storage_addr_t addr, void *buf,
    storage_len_t len, error_t err)
  {
    if(err == SUCCESS)
    {
      //now commit data
      error_t result = call Config.commit();
      
      if(result == EBUSY)
        signalWriteCompletion(EBUSY);
      else
        signalWriteCompletion(FAIL);
    }
    else
    {
      signalWriteCompletion(FAIL);
    }
  }

  event void Config.commitDone(error_t err)
  {
    if (err ==  SUCCESS)
    {
      signalWriteCompletion(SUCCESS);
    }
    else
    {
      signalWriteCompletion(FAIL);
    }
  }

  task void signalReadFailure()
  {
    signal BootConfigurator.configureDone(FAIL, config);
  }

  task void signalReadSuccess()
  {
    signal BootConfigurator.configureDone(SUCCESS, config);
  }

  task void signalReadTemporalError()
  {
    signal BootConfigurator.configureDone(EBUSY, config);
  }

  task void signalWriteFailure()
  {
    signal BootConfigurator.writeConfigDone(FAIL);
  }

  task void signalWriteSuccess()
  {
    signal BootConfigurator.writeConfigDone(SUCCESS);
  }

  task void signalWriteTemporalError()
  {
    signal BootConfigurator.writeConfigDone(EBUSY);
  }

  void signalReadCompletion(error_t err, config_data_t* ptr)
  {
    if (ptr == NULL)
    {
      //free allocated memory
      free(config);
      config = NULL;
    }

    if (err == FAIL)
      post signalReadFailure();
    else if(err == SUCCESS)
      post signalReadSuccess();
    else
      post signalReadTemporalError();
  }

  void signalWriteCompletion(error_t err)
  {
    if (err == SUCCESS)
      post signalWriteSuccess();
    else if(err == FAIL)
      post signalWriteFailure();
    else
      post signalWriteTemporalError();
  }
}

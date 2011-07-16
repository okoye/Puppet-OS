#include "PuppetConfiguration.h"

interface PuppetConfigurator
{
  /**
   *This should retrieve configuration files from disk if exists
   *If no configuration is found, it will return NULL in it associated
   *event handler.
   *it returns an error_t if a problem occured while trying to
   *initiate the configuration process
   */
  async command error_t configure();

  /**
   *This should write configuration files for permemnent storage
   *
   *@param config_data_t* data, data to be written to storage
   */
  async command void writeConfig(config_data_t* data);

  /**
   *Signaled after configure command finishes execution
   *
   *@return config_data_t* data, if config data was found, it returns
            the config data. Otherwise will return NULL. If an error 
            occured during configuration, err will be a FAIL otherwise
            it should be a success. Note, if no config data is present,
            it will return a SUCCESS but with NULL as data. It can also
            return EBUSY if a request is already being processed.
   */
  event void configureDone(error_t err, config_data_t* data);

  /**
   *Signaled after a writeConfig command has completed.
   * 
   *@return error_t error, SUCCESS if successful, otherwise FAIL
   */
  event void writeConfigDone(error_t err);

}

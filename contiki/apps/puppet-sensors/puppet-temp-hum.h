#ifndef PUPPET_TEMP_HUM_H_
#define PUPPET_TEMP_HUM_H_

/*Declare process*/
PROCESS_NAME(puppet_temp_hum);

//TODO: thresholding support (up&down) can be added as enhancement.

/*Type definition of temperature callback*/
typedef int (*temp_hum_callback) (unsigned temp, unsigned hum);

/*
 *Set temperature callback
 */
void set_temp_hum_callback(temp_hum_callback callback);

/*
 *Read temperature directly from sensor
 */
static
unsigned read_temp(void);

/*
 *Read humidity directly from sensor
 */
static
unsigned read_hum(void);

/*
 *Type definition for temp_hum_reading
 */
typedef struct th_reading
{
  unsigned temperature;
  unsigned humidity;
}temp_hum_reading;

/*
 *Accessofor retrieving temp_hum_reading
 *Buffer can be NULL in which case it checks if a callback
 *is registered.
 */
int read_temp_hum(temp_hum_reading* buffer);

//define READING_DURATION
#endif

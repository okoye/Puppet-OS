/*
 *Licensing information & copyright information
 *@author: Chuka
 *@email: chuka@puppetme.com
 */

enum{
  OUTPUT_BUFFER_SIZE = 100,
  POLL_INTERVAL = 7200, //Fires every 2 hrs.
};


/*Initialize processes, and other vars for temperature. Main entry point*/
void ptemperature_initialize();

/*Send a udp message to proxy*/
static
void send_data();

static
void clear_buffer();

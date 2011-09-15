/*
 *Licensing information & copyright information
 *@author: Chuka
 *@email: chuka@puppetme.com
 */

#define REMOTE_PORT 5688
#define LOCAL_PORT 8775

enum{
  OUTPUT_BUFFER_SIZE = 100,
  POLL_INTERVAL = 20, //7200 Fires every 2 hrs.
};


/*Initialize processes, and other vars for temperature. Main entry point*/
void ptemperature_initialize();

/*Send a udp message to proxy*/
static
void send_data();

/*Delete currently allocated buffer*/
static
void clear_buffer();

static
void generate_payload(char* buf, unsigned data);

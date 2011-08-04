#include "contiki.h"
#include "cfs/cfs.h"
#include <stdio.h>

PROCESS(filesaveprocess, "Coffee File Save Process");
AUTOSTART_PROCESSES(&filesaveprocess);

PROCESS_THREAD(filesaveprocess, events, data)
{
  PROCESS_BEGIN();
  char msg[32];
  char* filename = "initial.cfg";
  int fd_write, fd_read;
  int n;

  strcpy(msg,"#hello files");
  printf("Step 0: %s",msg);

  //writing to file system
  fd_write = cfs_open(filename, CFS_WRITE);
  if(fd_write != -1)
  {
    n = cfs_write(fd_write,msg, sizeof(msg));
    cfs_close(fd_write);
    printf("Step 1: Successfully written data to disk\n");
  }else{
    printf("ERROR: Failed to write data to disk\n");
  }
  //reading from filesystem
  strcpy(msg,"");
  fd_read = cfs_open(filename, CFS_READ);
  if(fd_read != -1){
    cfs_read(fd_read, msg, sizeof(msg));
    cfs_close(fd_read);
    printf("Step 2: Successfully read %s from disk\n",msg);
  }else{
    printf("ERROR: Failed to read data from disk\n");
  }
  //appending to filesystem
  //TODO
  //Seeking specific data

  PROCESS_END();
}

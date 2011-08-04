#include "contiki.h"
#include "cfs/cfs.h"

int fd_write, fd_read;
static int 
writeData(char* filename, void* data, unsigned int len){
  //Open filename indicated and read element of size len.
  //If it encounters an error, it returns a NULL object.
  fd_write = cfs_open(filename, CFS_WRITE);
  if(fd_write != 1){
    cfs_write(fd_write, data, len);
    cfs_close(fd_write);
    return 0;
  }else{
    return -1;
  }
}

static void
readData(char* filename, void* buf, unsigned int len){
  fd_read = cfs_open(filename, CFS_READ);
  if(fd_read != -1){
    cfs_read(fd_read, buf, len);
    cfs_close(fd_read);
  }else{
    buf = NULL;
  }
}

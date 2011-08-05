#include "contiki.h"
#include "filemanager.h"
#include <stdio.h>

PROCESS(filesaveprocess, "Test File Save Process");
AUTOSTART_PROCESSES(&filesaveprocess);

PROCESS_THREAD(filesaveprocess, ev, data){
  PROCESS_BEGIN();
  char* msg;
  unsigned int counter = 1;
  struct complex_structure
  {
    unsigned int value;
    char* string;
  };
  msg = "ChukaBaka";
  struct complex_structure mystruct;
  mystruct.value = 2;
  mystruct.string = "Chuka";

  printf("Write 1 Returned %d\n",writeData("file1",msg,sizeof(char[10])));
  printf("Write 2 Returned %d\n",writeData("file2",&counter,sizeof(int)));
  printf("Write 3 Returned %d\n",writeData("file3",&mystruct,
                          sizeof(struct complex_structure)));

  msg = "";
  readData("file1",msg,sizeof(char[10]));
  printf("Read 1 Returned %s\n",msg);
  counter = 0;
  readData("file2",&counter,sizeof(int));
  printf("Read 2 Returned %d\n",counter);
  struct complex_structure test;
  readData("file3",&test,sizeof(struct complex_structure));
  printf("Read 3 Returned %d %s\n",test.value,test.string);
    
  PROCESS_END();
}

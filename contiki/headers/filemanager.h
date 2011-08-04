
//Writes 'data' of size 'len' to filename specified by
//'filename'. If an error is encountered, it returns -1,
//otherwise it should return 0.
static int
writeData(char* filename, void* data, unsigned int len);


//Reads into 'buf' the contents of 'filename' of size 'len'.
//If an error is encountered, it returns NULL as the content
//of buf.
static void
readData(char* filename, void* buf, unsigned int len);

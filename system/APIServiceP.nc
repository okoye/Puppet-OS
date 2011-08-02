#include <message.h>
#include <APIService.h>

module APIServiceP{
  provides{
    interface APIService;
  }
  uses{
    interface UDP as NetworkService;
    interface Leds;
  }
}
implementation{
  /********************************************************
  *            Global Variables Here
  ********************************************************/
  struct sockaddr_in6 sink;
  bool initialized = FALSE;
  p_message_t msg;
  /********************************************************
  *           Method Definitions
  ********************************************************/
  error_t validateRegisterRequest(register_request_t* reg);
  void initializeSocket();
  void logError(char* msg);
  void registerHandler(struct sockaddr_in6 *f, void* b,
                        uint16_t l, struct ip_metadata *m);
  /*******************************************************
  *             Command Implementations
  ********************************************************/
  command error_t registerRequest(register_request_t* reg){
    //Take the registration request supplied and validate
    //that its contents are valid. If so, proceed to send the 
    //data using UDP interface after encapsulating it in our
    //standard data packet.
    error_t err = validateRegisterRequest(reg);
    if(err != SUCCESS){
      return err;
    }else{
      if(!initialized)
        initializeSocket();
      msg.resource_url = REGISTER_URL;
      msg.http_method = "POST";
      msg.version = 1;
      msg.body = reg;
      return call NetworkService.sendto(&sink, &msg, sizeof(p_message_t));
    }
  }
  /********************************************************
  *               Event Implementations
  *********************************************************/
  event void NetworkService.recvfrom(struct sockaddr_in6 *src, void *payload,
                                uint16_t len, struct ip_metadata *meta){
    //First verify the data is from SINK node, after verification,
    //Check url in switch statement and handle in appropriate 
    //handler.
    if(*src == *sink){
      switch(((p_message*)payload)->resource_url){
        case REGISTER_URL:
          //call register handler
          break;
        default:
          logError("Unhandled request message type");
          break;
      }
    }else{
      logError("Message received not from SINK node.");
    }
  }
  /********************************************************
  *               Method Implementations
  *********************************************************/
  error_t validateRegisterRequest(register_request_t* reg){
    if(reg->device_type_id != NULL &&
        reg->sensor_ids != NULL && reg->man_id != NULL){
      return SUCCESS;
    }else{
      return EINVAL;
    }
  }
  void initializeSocket(){
    //Initialize address to send data to sink node
    //also initialize socket for listening data from
    //sink node.
    memset(&sink,0,sizeof(struct sockaddr_in6));
    sink.sin6_addr.s6_addr16[0] = htons(SINK_ADDRESS_PREFIX);
    sink.sin6_addr.s6_addr[15] = SINK_ADDRESS_SUFFIX;
    sink.sin6_port = htons(SINK_ADDRESS_PORT);
  }
  void logError(char* msg){
    //Handles how error messages are logged.
    Leds.led0Toggle();
  }
  void registerHandler(struct sockaddr_in6 *from, void* data,
                        uint16_t len, struct ip_metadata *meta){
    Signal APIService.registerResponse(((p_response_t*)data)->body,
                                        ((p_response_t*)data)->http_code);
  }
}

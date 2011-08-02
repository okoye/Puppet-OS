configuration APIServiceC{
  provides{
    interface SplitControl;
    interface APIService;
  }
}
implementation{
  components IPDispatchC, new UdpSocketC(), LedsC;
  components APIServiceP;

  SplitControl = IPDispatch.SplitControl;
  APIService = APIServiceP;

  APIServiceP.NetworkService -> UdpSocketC;
  APIServiceP.Leds -> LedsC;
}

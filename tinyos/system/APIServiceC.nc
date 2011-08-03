configuration APIServiceC{
  provides{
    interface SplitControl;
    interface APIService;
  }
}
implementation{
  components IPDispatchC, new UdpSocketC(), LedsC;
  components APIServiceP;

  SplitControl = IPDispatchC.SplitControl;
  APIService = APIServiceP;

  APIServiceP.NetworkService -> UdpSocketC;
  APIServiceP.Leds -> LedsC;
}

configuration BlinkC
{}
implementation
{
  components MainC, BlinkP, LedsC;
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components new TimerMilliC() as Timer2;

  BlinkP -> MainC.Boot;

  BlinkP.Timer0 -> Timer0;
  BlinkP.Timer1 -> Timer1;
  BlinkP.Timer2 -> Timer2;
  BlinkP.Leds -> LedsC;
}

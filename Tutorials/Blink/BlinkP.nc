#include "Timer.h"
module BlinkP
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Timer<TMilli> as Timer1;
  uses interface Timer<TMilli> as Timer2;
  uses interface Leds;
  uses interface Boot;
}
implementation
{
  event void Boot.booted()
  {
    call Timer0.startPeriodic(250);
    call Timer1.startPeriodic(500);
    call Timer2.startPeriodic(1000);
  }

  event void Timer0.fired()
  {
    dbg("logging","Timer0 fired @ %s\n",sim_time_string());
  }

  event void Timer1.fired()
  {
    dbg("logging","Timer1 fired @ %s\n",sim_time_string());
  }

  event void Timer2.fired()
  {
    dbg("logging","Timer2 fired @ %s\n",sim_time_string());
  }
}

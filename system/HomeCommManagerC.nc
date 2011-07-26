/**
  *@author: Chuka Okoye
  *@email: chuka@puppetme.com
**/

configuration HomeCommManagerC
{
  provides
  {
    interface SplitControl;
    interface PuppetAPI;
  }
}

implementation
{
  //TODO: put some wirings here
  components HomeCommManagerP;
  components ActiveMessageC as Radio;

  PuppetAPI = HomeCommManagerP;
  SplitControl = HomeCommManagerP;

  HomeCommManagerP.RadioControl -> Radio;
}

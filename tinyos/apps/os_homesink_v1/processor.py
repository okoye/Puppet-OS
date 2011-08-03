#!/usr/bin/env python

from tos import getSource
from tos import AM
from packets.Register import RegisterRequest

AM_REGISTER_REQUEST = 0x10

class PacketProcessor(object):
  
  def __init__(self):
    self.debug = True
    self.source = getSource("serial@/dev/ttyUSB0:115200") #TODO: Re-implement.
    self.am = AM(s=self.source)

  def run(self):
    p = None
    while True:
      try:
        p = self.am.read(timeout=10)
      except Exception as e:
        self.log("Error:\t%s"%e)
        raise
      finally:
        if p and p.type == AM_REGISTER_REQUEST:
          reg_message = RegisterRequest(data=p.data)
          self.log(reg_message)

  def log(self, msg):
    if self.debug:
      print msg

if __name__ == '__main__':
  PacketProcessor().run()

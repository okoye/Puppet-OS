#!/usr/bin/env python

import tos
from packets.Register import RegisterRequest

AM_REGISTER_REQUEST = 0x10

class PacketProcessor(object):
  
  def __init__(self):
    self.am = tos.AM()
    self.debug = True

  def run(self):
    while True:
      if p and p.type == AM_REGISTER_REQUEST:
        reg_message = RegisterRequest(data=p.data)
        self.log(reg_message)

  def log(self, msg):
    if self.debug:
      print msg

if __name__ == '__main__':
  PacketProcessor().run()

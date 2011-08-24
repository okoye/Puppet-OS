#Stateful proxy for COAP
#The proxy mimicks the methods available on the webserver
#and provides some extra functions.
#@author: Chuka

import coapy
import optparse

#Optional args include:
parser = optparse.OptionParser()
parser.add_option('-p', '--port', help='port number', dest='port',
  default=5688)
parser.add_option('-a', '--address', help='Listen address',dest='address',
  default=
(opts,args) = parser.parse_args()

port = opts.port

bindAddr = ('::', port, 0, 0)
endPoint = coapy.connection.EndPoint(address_family=address_family)
endPoint.bind(bindAddr)
endPoint.bindDiscovery(opts.address)

'''1-1 mapping on sensor api method definitions'''
class 

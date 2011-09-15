#Stateless Router for COAP-based Applications
#and provides some extra functions.
#@author: Chuka

import coapy.connection
import coapy.options
import coapy.link
import optparse, socket
from urllib import urlencode
from httplib import HTTPConnection

#Optional args include:
parser = optparse.OptionParser()
parser.add_option('-p', '--port', help='port number', dest='port',
  default=5688)
parser.add_option('-a', '--address', help='Listen address',dest='address',
  default='aaaa::1')
parser.add_option('-d', '--debug', help="Debug switch.(true)",dest='debug',
  default=True)
(opts,args) = parser.parse_args()

port = opts.port

bindAddr = (opts.address, port, 0, 0)
endPoint = coapy.connection.EndPoint(address_family=socket.AF_INET6)
if opts.debug:
  print 'binding to %s'%str(bindAddr)
endPoint.bind(bindAddr)
endPoint.bindDiscovery(opts.address)


'''1-1 mapping on sensor api method definitions'''
class ProxyService(coapy.link.LinkValue):
  
  def _getHandler(self, uri,data=None):
    if data:
      data = [lambda x: tuple(x.split('=')) for chunk in data.split(',')]
    print data

  def _postHandler(self, uri,data=None):
    args = dict()
    if data:
      data = [lambda x: tuple(x.split('=')) for chunk in data.split(',')]
    for key,value in data:
      args[key] = value
    print uri,args

  def _putHandler(self, uri,data=None):
    args = dict()
    if data:
      data = [lambda x: tuple(x.split('=')) for chunk in data]

  def process(self, record):
    '''
    Examine the contents of Proxy-Uri and execute requested action.
    '''
    rcv_msg = record.message
    proxy_url = rcv_msg.findOption(coapy.options.ProxyUri)
    resp = None 
    if not proxy_url:
      #Bad request.
      msg = coapy.connection.Message(coapy.connection.Message.ACK,
              code=coapy.BAD_REQUEST)
      return record.ack(msg)
    else:
      if rcv_msg.code == coapy.GET:
        resp = self._getHandler(proxy_url.value,rcv_msg.payload)
      elif rcv_msg.code == coapy.POST:
        resp = self._postHandler(proxy_url.value,rcv_msg.payload)
      elif rcv_msg.code == coapy.PUT:
        resp = self._putHandler(proxy_url.value,rcv_msg.payload)
      else:
        record.ack(coapy.connection.Message(coapy.connection.Message.ACK,
          code=coapy.METHOD_NOT_ALLOWED))
      msg = coapy.connection.Message(coapy.connection.Message.ACK,
            code=coapy.OK,
            payload=resp)
      return record.ack(msg)

class PollListenerService(coapy.link.LinkValue):

  def process(self, record):
    '''
    Continuously polls service for messages.
    '''
    pass

class CometListenerService(coapy.link.LinkValue):

  def process(self, record):
    '''
    For each registered device, connect to comet server
    and retrieve messages.
    '''
    pass

class DiscoveryService(coapy.link.LinkValue):
  
  _services = None

  def __init__(self, *args, **kwargs):
    super(DiscoveryService,self).__init__('.well-known/resource',
      ct=[coapy.media_types_rev.get('application/link-format')])
    self._services = {self.uri:self}

  def add_service(self, service):
    self._services[service.uri] = service

  def lookup(self, uri):
    return self._services.get(uri,None)

  def process(self, record):
    msg = coapy.connection.Message(coapy.connection.Message.ACK, code=coapy.OK,
            content_type='application/link-format')
    msg.payload = ','.join([service.encode() for service in
    self._services.itervalues()])
    record.ack(msg)


#Add all services to service list.
services = DiscoveryService()
services.add_service(ProxyService('proxy'))
services.add_service(CometListenerService('comet'))
services.add_service(PollListenerService('poll'))

while True:
  record = endPoint.process(20000)
  if record is None:
    print '.'
    continue
  if opts.debug:
    print 'Message from %s: %s'%(record.remote,record.message)
  uri = record.message.findOption(coapy.options.UriPath)
  if not uri:
    continue
  else:
    service = services.lookup(uri.value)
    if opts.debug:
      print 'Executing %s for %s'%(uri.value,service)
    if service is None:
      record.reset()
      continue
    service.process(record)

#
# This class is automatically generated by mig. DO NOT EDIT THIS FILE.
# This class implements a Python interface to the 'RegisterRequest'
# message type.
#

import tinyos.message.Message

# The default size of this message type in bytes.
DEFAULT_MESSAGE_SIZE = 24

# The Active Message type associated with this message.
AM_TYPE = 16

class RegisterRequest(tinyos.message.Message.Message):
    # Create a new RegisterRequest of size 24.
    def __init__(self, data="", addr=None, gid=None, base_offset=0, data_length=24):
        tinyos.message.Message.Message.__init__(self, data, addr, gid, base_offset, data_length)
        self.amTypeSet(AM_TYPE)
    
    # Get AM_TYPE
    def get_amType(cls):
        return AM_TYPE
    
    get_amType = classmethod(get_amType)
    
    #
    # Return a String representation of this message. Includes the
    # message type name and the non-indexed field values.
    #
    def __str__(self):
        s = "Message <RegisterRequest> \n"
        try:
            s += "  [device_type_id=0x%x]\n" % (self.get_device_type_id())
        except:
            pass
        try:
            s += "  [sensor_ids=0x%x]\n" % (self.get_sensor_ids())
        except:
            pass
        try:
            s += "  [man_id=0x%x]\n" % (self.get_man_id())
        except:
            pass
        return s

    # Message-type-specific access methods appear below.

    #
    # Accessor methods for field: device_type_id
    #   Field type: long
    #   Offset (bits): 0
    #   Size (bits): 32
    #

    #
    # Return whether the field 'device_type_id' is signed (False).
    #
    def isSigned_device_type_id(self):
        return False
    
    #
    # Return whether the field 'device_type_id' is an array (False).
    #
    def isArray_device_type_id(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'device_type_id'
    #
    def offset_device_type_id(self):
        return (0 / 8)
    
    #
    # Return the offset (in bits) of the field 'device_type_id'
    #
    def offsetBits_device_type_id(self):
        return 0
    
    #
    # Return the value (as a long) of the field 'device_type_id'
    #
    def get_device_type_id(self):
        return self.getUIntElement(self.offsetBits_device_type_id(), 32, 1)
    
    #
    # Set the value of the field 'device_type_id'
    #
    def set_device_type_id(self, value):
        self.setUIntElement(self.offsetBits_device_type_id(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'device_type_id'
    #
    def size_device_type_id(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'device_type_id'
    #
    def sizeBits_device_type_id(self):
        return 32
    
    #
    # Accessor methods for field: sensor_ids
    #   Field type: long
    #   Offset (bits): 64
    #   Size (bits): 64
    #

    #
    # Return whether the field 'sensor_ids' is signed (False).
    #
    def isSigned_sensor_ids(self):
        return False
    
    #
    # Return whether the field 'sensor_ids' is an array (False).
    #
    def isArray_sensor_ids(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'sensor_ids'
    #
    def offset_sensor_ids(self):
        return (64 / 8)
    
    #
    # Return the offset (in bits) of the field 'sensor_ids'
    #
    def offsetBits_sensor_ids(self):
        return 64
    
    #
    # Return the value (as a long) of the field 'sensor_ids'
    #
    def get_sensor_ids(self):
        return self.getUIntElement(self.offsetBits_sensor_ids(), 64, 0)
    
    #
    # Set the value of the field 'sensor_ids'
    #
    def set_sensor_ids(self, value):
        self.setUIntElement(self.offsetBits_sensor_ids(), 64, value, 0)
    
    #
    # Return the size, in bytes, of the field 'sensor_ids'
    #
    def size_sensor_ids(self):
        return (64 / 8)
    
    #
    # Return the size, in bits, of the field 'sensor_ids'
    #
    def sizeBits_sensor_ids(self):
        return 64
    
    #
    # Accessor methods for field: man_id
    #   Field type: long
    #   Offset (bits): 128
    #   Size (bits): 32
    #

    #
    # Return whether the field 'man_id' is signed (False).
    #
    def isSigned_man_id(self):
        return False
    
    #
    # Return whether the field 'man_id' is an array (False).
    #
    def isArray_man_id(self):
        return False
    
    #
    # Return the offset (in bytes) of the field 'man_id'
    #
    def offset_man_id(self):
        return (128 / 8)
    
    #
    # Return the offset (in bits) of the field 'man_id'
    #
    def offsetBits_man_id(self):
        return 128
    
    #
    # Return the value (as a long) of the field 'man_id'
    #
    def get_man_id(self):
        return self.getUIntElement(self.offsetBits_man_id(), 32, 1)
    
    #
    # Set the value of the field 'man_id'
    #
    def set_man_id(self, value):
        self.setUIntElement(self.offsetBits_man_id(), 32, value, 1)
    
    #
    # Return the size, in bytes, of the field 'man_id'
    #
    def size_man_id(self):
        return (32 / 8)
    
    #
    # Return the size, in bits, of the field 'man_id'
    #
    def sizeBits_man_id(self):
        return 32
    

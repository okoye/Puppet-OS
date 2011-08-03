COMPONENT=TestAPIRegisterC

OS_HOME = /home/chuka/projects/puppet-os

#puppet libraries specific stuff
CFLAGS += -I$(OS_HOME)/system/
CFLAGS += -L$(OS_HOME)/system/
CFLAGS += -I$(OS_HOME)/interfaces/
CFLAGS += -I$(OS_HOME)/types/
CFLAGS += -I$(TOSDIR)/lib/printf

#ipv6 specific stuff
CFLAGS += -DCC2420_DEF_CHANNEL=15
CFLAGS += -DIEEE154FRAMES_ENABLED
include $(MAKERULES)


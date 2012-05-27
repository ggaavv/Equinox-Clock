# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)

# Local flags
CFLAGS_$(d) := $(WIRISH_INCLUDES) $(LIBMAPLE_INCLUDES)

# Local rules and targets
cSRCS_$(d) :=   network.c                      \
                psock.c                        \
                uiplib.c                       \
                uip-neighbor.c                 \
                clock-arch.c                   \
                timer.c                        \
                uip-split.c                    \
                strings.c                      \
                memb.c                         \
                uip-fw.c                       \
                g2100.c                        \
                uip_arp.c                      \
                stack.c                        \
                uip.c                          \
                webserver.c

#                 examples/UDPApp/udpapp.c       \
#                 examples/SocketApp/socketapp.c \
#                 examples/WebClient/webclient.c \
#                 examples/WebServer/webserver.c \
#                 examples/Flash/webserver.c     \

cppSRCS_$(d) := WiShield.cpp \
	        WiServer.cpp \
		request.cpp

cFILES_$(d) := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)

OBJS_$(d)     := $(cFILES_$(d):%.c=$(BUILD_PATH)/%.o) \
                 $(cppFILES_$(d):%.cpp=$(BUILD_PATH)/%.o)
DEPS_$(d)     := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include        $(DEPS_$(d))
d               := $(dirstack_$(sp))
sp              := $(basename $(sp))

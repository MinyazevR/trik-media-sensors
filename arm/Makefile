#
#  Copyright (c) 2013-2018 Texas Instruments Incorporated - http://www.ti.com
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  *  Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#  *  Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  *  Neither the name of Texas Instruments Incorporated nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
#  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

srcs = src/main.c src/args.c src/arm_server.c src/module_fb.c src/module_rc.c src/module_v4l2.c src/runtime.c src/thread_input.c src/thread_video.c

EXBASE = ..
include $(EXBASE)/products.mak
-include $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A.dep,$(srcs)))

objs = $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.ov7A,$(srcs)))

ifdef LIBS_STATIC
libs = libtitransportrpmsg.a \
       libtiipc.a \
       libtiipcutils.a
else
# Use dynamic or shared libs
#libs = $(IPC_INSTALL_DIR)/linux/src/api/.libs/libtiipc.so \
#       $(IPC_INSTALL_DIR)/linux/src/utils/.libs/libtiipcutils.so \
#       $(IPC_INSTALL_DIR)/linux/src/transport/.libs/libtitransportrpmsg.so
libs =
# Use dynamic or static libraries installed on your linux distribution
endif

#  ======== install validation ========
ifeq (install,$(MAKECMDGOALS))
ifeq (,$(EXEC_DIR))
$(error must specify EXEC_DIR)
endif
endif

#  ======== toolchain macros ========
CFLAGS = -Iinclude/ -I../shared/include
CPPFLAGS = -Iinclude/ -I../shared/include
ifndef LINUX_SYSROOT_DIR
SDKTARGETSYSROOT=$(TRIK_SDK_DIR)/sysroots/arm926ejse-oe-linux-gnueabi
CC = $(TOOLCHAIN_PREFIX)gcc -marm -mcpu=arm926ej-s --sysroot=$(SDKTARGETSYSROOT)
AR = $(TOOLCHAIN_PREFIX)ar
LD = $(TOOLCHAIN_PREFIX)gcc --sysroot=$(SDKTARGETSYSROOT)
LDFLAGS = -L$(IPC_INSTALL_DIR)/linux/src/api/.libs/ \
    -L$(IPC_INSTALL_DIR)/linux/src/utils/.libs \
    -L$(IPC_INSTALL_DIR)/linux/src/transport/.libs
else
LD = $(CC)
endif

CFLAGS += -c
ARFLAGS = cr

CPPFLAGS += -D_REENTRANT

CFLAGS += -Wall -ffloat-store -fPIC -Wunused -pthread -Dfar= $(CCPROFILE_$(PROFILE)) \
    -I. -I..
ifdef LINUX_SYSROOT_DIR
CFLAGS += -I$(LINUX_SYSROOT_DIR)
else
CFLAGS += -I$(IPC_INSTALL_DIR)/linux/include -I$(IPC_INSTALL_DIR)/packages
endif

LDFLAGS += $(LDPROFILE_$(PROFILE)) -Wall -Wl,-Map=$@.map

LDLIBS = -lpthread -lc -lrt
ifndef LIBS_STATIC
LDLIBS +=-ltiipc -ltiipcutils -ltitransportrpmsg
endif
CCPROFILE_debug = -ggdb -D DEBUG #-fsanitize=address -fsanitize=undefined
CCPROFILE_release = -O3 -D NDEBUG

LDPROFILE_debug = -L$(LINUX_SYSROOT_DIR) -ggdb #-lasan -lubsan
LDPROFILE_release = -O3

#  ======== standard macros ========
ifneq (,$(wildcard $(XDC_INSTALL_DIR)/bin/echo.exe))
    # use these on Windows
    CP      = $(XDC_INSTALL_DIR)/bin/cp
    ECHO    = $(XDC_INSTALL_DIR)/bin/echo
    MKDIR   = $(XDC_INSTALL_DIR)/bin/mkdir -p
    RM      = $(XDC_INSTALL_DIR)/bin/rm -f
    RMDIR   = $(XDC_INSTALL_DIR)/bin/rm -rf
else
    # use these on Linux
    CP      = cp
    ECHO    = echo
    MKDIR   = mkdir -p
    RM      = rm -f
    RMDIR   = rm -rf
endif

#  ======== create output directories ========
ifneq (clean,$(MAKECMDGOALS))
ifneq (,$(PROFILE))
ifeq (,$(wildcard bin/$(PROFILE)/obj))
    $(shell $(MKDIR) -p bin/$(PROFILE)/obj)
endif
endif
endif

all:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(MAKE) PROFILE=debug app_host
	$(MAKE) PROFILE=release app_host

help:
	@$(ECHO) "make                  # build executables"
	@$(ECHO) "make clean            # clean everything"

install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/debug
	$(CP) bin/debug/app_host $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/release/app_host $(EXEC_DIR)/release

clean::
	$(RMDIR) bin


#
#  ======== rules ========
#
app_host: bin/$(PROFILE)/app_host
bin/$(PROFILE)/app_host: $(objs) $(libs)
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@-$(MKDIR) $(dir $@)
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)

bin/$(PROFILE)/obj/%.ov7A: %.c
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@-$(MKDIR) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $<
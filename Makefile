#############################################################################
#
# Makefile for MySensors
#
#  
# The arduino library build part were inspired by
#   Arduino-Makefile project, Copyright (C) 2012 Sudar <http://sudarmuthu.com>
#
# Description:
# ------------
# use make all and make install to install the gateway
#

CONFIG_FILE=Makefile.inc

include $(CONFIG_FILE)

CPPFLAGS+=-Ofast -g -Wall -Wextra
DEPFLAGS=-MT $@ -MMD -MP

GATEWAY_BIN=$(SERVICE_NAME)
GATEWAY=$(BINDIR)/$(GATEWAY_BIN)
GATEWAY_C_SOURCES=$(wildcard hal/architecture/Linux/drivers/core/*.c)
GATEWAY_CPP_SOURCES=$(wildcard hal/architecture/Linux/drivers/core/*.cpp) examples_linux/mysgw.cpp
GATEWAY_OBJECTS=$(patsubst %.c,$(BUILDDIR)/%.o,$(GATEWAY_C_SOURCES)) $(patsubst %.cpp,$(BUILDDIR)/%.o,$(GATEWAY_CPP_SOURCES))

INCLUDES=-I. -I./core -I./hal/architecture/Linux/drivers/core

ifeq ($(SOC),$(filter $(SOC),BCM2835 BCM2836 BCM2837 BCM2711))
BCM_C_SOURCES=$(wildcard hal/architecture/Linux/drivers/BCM/*.c)
BCM_CPP_SOURCES=$(wildcard hal/architecture/Linux/drivers/BCM/*.cpp)
GATEWAY_OBJECTS+=$(patsubst %.c,$(BUILDDIR)/%.o,$(BCM_C_SOURCES)) $(patsubst %.cpp,$(BUILDDIR)/%.o,$(BCM_CPP_SOURCES))

INCLUDES+=-I./hal/architecture/Linux/drivers/BCM
endif

# Gets include flags for library
get_library_includes = $(if $(and $(wildcard $(1)/src), $(wildcard $(1)/library.properties)), \
							-I$(1)/src, \
							$(addprefix -I,$(1) $(wildcard $(1)/utility)))

# Gets all sources with given extension (param2) for library (path = param1)
# for old (1.0.x) layout looks in . and "utility" directories
# for new (1.5.x) layout looks in src and recursively its subdirectories
get_library_files  = $(if $(and $(wildcard $(1)/src), $(wildcard $(1)/library.properties)), \
						$(call rwildcard,$(1)/src/,*.$(2)), \
						$(wildcard $(1)/*.$(2) $(1)/utility/*.$(2)))

ifdef ARDUINO_LIB_DIR
ARDUINO=arduino
ARDUINO_LIBS:=$(shell find $(ARDUINO_LIB_DIR) -mindepth 1 -maxdepth 1 -type d)
ARDUINO_INCLUDES:=$(foreach lib, $(ARDUINO_LIBS), $(call get_library_includes,$(lib)))
ARDUINO_LIB_CPP_SRCS:=$(foreach lib, $(ARDUINO_LIBS), $(call get_library_files,$(lib),cpp))
ARDUINO_LIB_C_SRCS:=$(foreach lib, $(ARDUINO_LIBS), $(call get_library_files,$(lib),c))
ARDUINO_LIB_AS_SRCS:=$(foreach lib, $(ARDUINO_LIBS), $(call get_library_files,$(lib),S))
ARDUINO_LIB_OBJS=$(patsubst $(ARDUINO_LIB_DIR)/%.cpp,$(BUILDDIR)/arduinolibs/%.cpp.o,$(ARDUINO_LIB_CPP_SRCS)) \
				$(patsubst $(ARDUINO_LIB_DIR)/%.c,$(BUILDDIR)/arduinolibs/%.c.o,$(ARDUINO_LIB_C_SRCS)) \
				$(patsubst $(ARDUINO_LIB_DIR)/%.S,$(BUILDDIR)/arduinolibs/%.S.o,$(ARDUINO_LIB_AS_SRCS))

INCLUDES+=$(ARDUINO_INCLUDES)
DEPS+=$(ARDUINO_LIB_OBJS:.o=.d)
endif

DEPS+=$(GATEWAY_OBJECTS:.o=.d)

.PHONY: all createdir cleanconfig clean install uninstall

all: createdir $(ARDUINO) $(GATEWAY)

createdir:
	@mkdir -p $(BUILDDIR) $(BINDIR)

# Arduino libraries Build
$(ARDUINO): CPPFLAGS+=-DARDUINO=100
$(ARDUINO): $(ARDUINO_LIB_OBJS)
	@printf "[Done building Arduino Libraries]\n"

# Gateway Build
$(GATEWAY): $(GATEWAY_OBJECTS) $(ARDUINO_LIB_OBJS)
	$(CXX) $(LDFLAGS) -o $@ $(GATEWAY_OBJECTS) $(ARDUINO_LIB_OBJS)

# Include all .d files
-include $(DEPS)

$(BUILDDIR)/arduinolibs/%.cpp.o: $(ARDUINO_LIB_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(DEPFLAGS) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILDDIR)/arduinolibs/%.c.o: $(ARDUINO_LIB_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILDDIR)/arduinolibs/%.S.o: $(ARDUINO_LIB_DIR)/%.S
	@mkdir -p $(dir $@)
	$(CC) $(DEPFLAGS) $(CPPFLAGS) $(ASFLAGS) $(INCLUDES) -c $< -o $@

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(DEPFLAGS) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILDDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(DEPFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -c $< -o $@

# clear configuration files
cleanconfig:
	@echo "[Cleaning configuration]"
	rm -rf $(CONFIG_FILE)

# clear build files
clean:
	@echo "[Cleaning]"
	rm -rf $(BUILDDIR) $(BINDIR)

$(CONFIG_FILE):
	@echo "[Running configure]"
	@./configure --no-clean

install: all install-gateway install-initscripts

install-gateway: 
	@echo "Installing $(GATEWAY) to ${DESTDIR}$(GATEWAY_DIR)"
	@install -m 0755 $(GATEWAY) ${DESTDIR}$(GATEWAY_DIR)

install-initscripts:
ifeq ($(INIT_SYSTEM), systemd)
	install -m0644 initscripts/mysgw.systemd ${DESTDIR}/etc/systemd/system/${SERVICE_NAME}.service
	@sed -i -e "s|%gateway_dir%|${GATEWAY_DIR}|g" ${DESTDIR}/etc/systemd/system/${SERVICE_NAME}.service
	@sed -i -e "s|%mysgw%|${SERVICE_NAME}|g" ${DESTDIR}/etc/systemd/system/${SERVICE_NAME}.service
	systemctl daemon-reload
	@echo "MySensors gateway has been installed, to add to the boot run:"
	@echo "  sudo systemctl enable ${SERVICE_NAME}.service"
	@echo "To start the gateway run:"
	@echo "  sudo systemctl start ${SERVICE_NAME}.service"
else ifeq ($(INIT_SYSTEM), sysvinit)
	install -m0755 initscripts/mysgw.sysvinit ${DESTDIR}/etc/init.d/${SERVICE_NAME}
	@sed -i -e "s|%gateway_dir%|${GATEWAY_DIR}|g" ${DESTDIR}/etc/init.d/${SERVICE_NAME}
	@sed -i -e "s|%mysgw%|${SERVICE_NAME}|g" ${DESTDIR}/etc/init.d/${SERVICE_NAME}
	@echo "MySensors gateway has been installed, to add to the boot run:"
	@echo "  sudo update-rc.d ${SERVICE_NAME} defaults"
	@echo "To start the gateway run:"
	@echo "  sudo service ${SERVICE_NAME} start"
endif

uninstall:
ifeq ($(INIT_SYSTEM), systemd)
	@echo "Stopping daemon ${SERVICE_NAME} (ignore errors)"
	-@systemctl stop ${SERVICE_NAME}.service
	@echo "removing files"
	rm /etc/systemd/system/${SERVICE_NAME}.service $(GATEWAY_DIR)/$(GATEWAY_BIN)
else ifeq ($(INIT_SYSTEM), sysvinit)
	@echo "Stopping daemon ${SERVICE_NAME} (ignore errors)"
	-@service ${SERVICE_NAME} stop
	@echo "removing files"
	rm /etc/init.d/${SERVICE_NAME} $(GATEWAY_DIR)/$(GATEWAY_BIN)
endif

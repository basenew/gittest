#
# File      : Makefile
# This file is part of atris-app
#
# Change Logs:
# Date           Author       Notes
# 2018-06-14     Pinkie.Fu    initial version
#

SHELL := /bin/bash

TOP=.

SSH_IP=127.0.0.1

TINY_ROS=$(PWD)/src/middleware/tinyros

DO_CHECK = if [ $$? != 0 ]; then exit 1; fi

DO_COMPILE = echo -e "\nGenerating atris-app..."; \
	python $(TINY_ROS)/scripts/make_library_gcc.py $(PWD) $(PWD)/devel/include/tinyros_client_library/gcc $(TINY_ROS); $(DO_CHECK); \
	python $(TINY_ROS)/scripts/make_library_rtthread.py $(PWD) $(PWD)/devel/include/tinyros_client_library/rtthread $(TINY_ROS); $(DO_CHECK); \
	python $(TINY_ROS)/scripts/make_library_java.py $(PWD) $(PWD)/devel/include/tinyros_client_library/java $(TINY_ROS); $(DO_CHECK); \
	python $(TINY_ROS)/scripts/make_library_python.py $(PWD) $(PWD)/devel/include/tinyros_client_library/python $(TINY_ROS); $(DO_CHECK); \
	python $(TINY_ROS)/scripts/make_library_go.py $(PWD) $(PWD)/devel/include/tinyros_client_library/go $(TINY_ROS); $(DO_CHECK); \
	python $(TINY_ROS)/scripts/make_library_js.py $(PWD) $(PWD)/devel/include/tinyros_client_library/js $(TINY_ROS); $(DO_CHECK); \
	source /opt/ros/melodic/setup.bash; catkin_make install; $(DO_CHECK); \
	mkdir -p $(PWD)/install/bin; \
	mkdir -p ${PWD}/install/firmware; \
	cp -rf $(PWD)/config $(PWD)/install; \
	cp -rf $(PWD)/scripts $(PWD)/install; \
	cp -rf $(PWD)/data $(PWD)/install; \
	cp -rf $(PWD)/tools/tinyrosnode/* $(PWD)/install/bin; \
	cp -rf $(PWD)/tools/tinyrosshutdown/* $(PWD)/install/bin; \
	cp -rf $(PWD)/tools/envtools/* $(PWD)/install/bin; \
	cp -rf $(PWD)/MCU/bin/* ${PWD}/install/firmware; \
	cd $(PWD)/install/bin; if [ -f "tinyrosdds" ]; then rm tinyrosdds; fi; ln -s ../lib/tinyros/tinyrosdds tinyrosdds; \
	cd $(PWD)/install/bin; if [ -f "tinyrostopic" ]; then rm tinyrostopic; fi; ln -s ../lib/tinyros/tinyrostopic tinyrostopic; \
	cd $(PWD)/install/bin; if [ -f "tinyrosservice" ]; then rm tinyrosservice; fi; ln -s ../lib/tinyros/tinyrosservice tinyrosservice; \
	cd $(PWD)/install/bin; if [ -f "tinyrosconsole" ]; then rm tinyrosconsole; fi; ln -s ../lib/tinyros/tinyrosconsole tinyrosconsole; \
	cd $(PWD)/install/bin; if [ -f "ExampleSubscriber" ]; then rm ExampleSubscriber; fi; ln -s ../lib/tinyros/ExampleSubscriber ExampleSubscriber; \
	cd $(PWD)/install/bin; if [ -f "ExampleServiceClient" ]; then rm ExampleServiceClient; fi; ln -s ../lib/tinyros/ExampleServiceClient ExampleServiceClient; \
	cd $(PWD)/install/bin; if [ -f "ExampleService" ]; then rm ExampleService; fi; ln -s ../lib/tinyros/ExampleService ExampleService; \
	cd $(PWD)/install/bin; if [ -f "ExamplePublisher" ]; then rm ExamplePublisher; fi; ln -s ../lib/tinyros/ExamplePublisher ExamplePublisher; \
	chmod -R 0777 $(PWD)/install/firmware; \
	chmod -R 0755 $(PWD)/install/scripts; \
	chmod -R 0755 $(PWD)/install/bin; \
	chmod -R 0755 $(PWD)/install/lib;

help:
	@echo -e "\nUsage: make [OPTION] [VAR=VALUE]"
	@echo -e "\nmake help:"
	@echo -e "       Display this help and exit"
	@echo -e "       This is the default command for make"
	@echo -e "\nmake debug ATRIS_FAMILY_NAME=<name>:"
	@echo -e "       ATRIS_FAMILY_NAME: [Atris1G], defualt: \"Atris1G\""
	@echo -e "       Example: \"make debug ATRIS_FAMILY_NAME=Atris1G\""
	@echo -e "\nmake clean:"
	@echo -e "       Clear source code compilation configuration"
	@echo -e "       Clear compiled production files"
	@echo -e "\nmake mcu:"
	@echo -e "       Build rt-thread project"
	@echo -e "\nmake mcuclean:"
	@echo -e "       Clear rt-thread project"
	@echo -e "\n\n"
debug:
	@$(DO_COMPILE)
cp2x86:
	@mkdir -p /home/atris/atris_app; $(DO_CHECK)
	@rm -rf /home/atris/atris_app; $(DO_CHECK)
	@cp -rpv $(PWD)/install /home/atris/atris_app; $(DO_CHECK)
mcu:
	@cd MCU/; ./build.sh
mcuclean:
	@cd MCU/; ./build.sh --clean
clean:
	@rm -rf $(PWD)/build
	@rm -rf $(PWD)/devel
	@rm -rf $(PWD)/install


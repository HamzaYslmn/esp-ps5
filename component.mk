#
# Component Makefile
#

COMPONENT_SRCDIRS := src src/bluedroid
COMPONENT_ADD_INCLUDEDIRS := src

COMPONENT_OBJS := src/ps5Controller.o src/ps5_bytes.o src/bluedroid/bluedroid.o

COMPONENT_DEPENDS := bt


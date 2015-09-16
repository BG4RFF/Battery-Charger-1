PROJECT =       BatteryCharger
TEMPLATE =      app
TARGET          += 
DEPENDPATH      += .
INCLUDEPATH     += ../qextserialport \
                   ../programmers/serial-programmer-pc/ \
                   ../acquisition/acquisition-lib/
QMAKE_LIBDIR    += ../qextserialport/build

OBJECTS_DIR     = obj
MOC_DIR         = moc
UI_HEADERS_DIR  = ui
UI_SOURCES_DIR  = ui
LANGUAGE        = C++
CONFIG          += qt warn_on release

LIBS            += -lqextserialport

# Input
FORMS           += batterychargercontroldialogue.ui \
                   batterychargerconfiguredialogue.ui \
                   batterychargeradvanceddialogue.ui \
                   batterychargercalibratedialogue.ui \
                   batterychargeraboutdialogue.ui \
                   ../acquisition/acquisition-lib/acqinfoblockdialogue.ui \
                   ../programmers/serial-programmer-pc/avrserialprog.ui \
                   ../programmers/serial-programmer-pc/m88Dialog.ui \
                   ../programmers/serial-programmer-pc/m48Dialog.ui \
                   ../programmers/serial-programmer-pc/m8535Dialog.ui \
                   ../programmers/serial-programmer-pc/m16Dialog.ui \
                   ../programmers/serial-programmer-pc/t261Dialog.ui \
                   ../programmers/serial-programmer-pc/s2313Dialog.ui \
                   ../programmers/serial-programmer-pc/t26Dialog.ui
HEADERS         += batterychargercontrol.h \
                   batterychargerconfigure.h \
                   batterychargeradvanced.h \
                   batterychargercalibrate.h \
                   batterychargerabout.h \
                   ../acquisition/acquisition-lib/acqinfoblock.h \
                   ../acquisition/acquisition-lib/acqpacket.h \
                   ../acquisition/acquisition-lib/acqgeneral.h \
                   ../acquisition/acquisition-lib/acqunit.h \
                   ../acquisition/acquisition-lib/acqsimpleadcunit.h \
                   ../acquisition/acquisition-lib/avrdevice.h \
                   ../programmers/serial-programmer-pc/serialport.h \
                   ../programmers/serial-programmer-pc/avrserialprog.h \
                   ../programmers/serial-programmer-pc/m88Dialog.h \
                   ../programmers/serial-programmer-pc/m48Dialog.h \
                   ../programmers/serial-programmer-pc/m8535Dialog.h \
                   ../programmers/serial-programmer-pc/m16Dialog.h \
                   ../programmers/serial-programmer-pc/t261Dialog.h \
                   ../programmers/serial-programmer-pc/t26Dialog.h   \
                   ../programmers/serial-programmer-pc/t2313Dialog.h  \
                   ../programmers/serial-programmer-pc/s2313Dialog.h
SOURCES         += batterychargermain.cpp \
                   batterychargercontrol.cpp \
                   batterychargerconfigure.cpp \
                   batterychargeradvanced.cpp \
                   batterychargercalibrate.cpp \
                   batterychargerabout.cpp \
                   ../acquisition/acquisition-lib/acqinfoblock.cpp \
                   ../acquisition/acquisition-lib/acqpacket.cpp \
                   ../acquisition/acquisition-lib/acqgeneral.cpp \
                   ../acquisition/acquisition-lib/acqunit.cpp \
                   ../acquisition/acquisition-lib/acqsimpleadcunit.cpp \
                   ../acquisition/acquisition-lib/avrdevice.cpp \
                   ../programmers/serial-programmer-pc/serialport.cpp \
                   ../programmers/serial-programmer-pc/avrserialprog.cpp \
                   ../programmers/serial-programmer-pc/m88Dialog.cpp \
                   ../programmers/serial-programmer-pc/m48Dialog.cpp \
                   ../programmers/serial-programmer-pc/m8535Dialog.cpp \
                   ../programmers/serial-programmer-pc/m16Dialog.cpp \
                   ../programmers/serial-programmer-pc/t261Dialog.cpp \
                   ../programmers/serial-programmer-pc/t26Dialog.cpp \
                   ../programmers/serial-programmer-pc/t2313Dialog.cpp \
                   ../programmers/serial-programmer-pc/s2313Dialog.cpp


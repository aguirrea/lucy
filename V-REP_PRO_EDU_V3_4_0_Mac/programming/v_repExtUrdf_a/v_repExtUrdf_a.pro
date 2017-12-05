# This file is part of the URDF PLUGIN for V-REP
#  
# Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# A big thanks to Ignacio Tartavull and Martin Pecka for their precious help!
# 
# The URDF PLUGIN is licensed under the terms of GNU GPL:
# 
# -------------------------------------------------------------------
# The URDF PLUGIN is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# THE URDF PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the URDF PLUGIN.  If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------

#
# This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

# The URDF plugin is courtesy of Ignacio Tartavull. A few modifications by Marc.

TARGET = v_repExtUrdf-a
TEMPLATE = lib

DEFINES -= UNICODE
CONFIG += shared
INCLUDEPATH += "../include"
INCLUDEPATH += "../v_repMath"
QT += widgets

*-msvc* {
	QMAKE_CXXFLAGS += -O2
	QMAKE_CXXFLAGS += -W3
}
*-g++* {
	QMAKE_CXXFLAGS += -O3
	QMAKE_CXXFLAGS += -Wall
	QMAKE_CXXFLAGS += -Wno-unused-parameter
	QMAKE_CXXFLAGS += -Wno-strict-aliasing
	QMAKE_CXXFLAGS += -Wno-empty-body
	QMAKE_CXXFLAGS += -Wno-write-strings

	QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
	QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
	QMAKE_CXXFLAGS += -Wno-narrowing

	QMAKE_CFLAGS += -O3
	QMAKE_CFLAGS += -Wall
	QMAKE_CFLAGS += -Wno-strict-aliasing
	QMAKE_CFLAGS += -Wno-unused-parameter
	QMAKE_CFLAGS += -Wno-unused-but-set-variable
	QMAKE_CFLAGS += -Wno-unused-local-typedefs
}


win32 {
    DEFINES += WIN_VREP
    INCLUDEPATH += "c:/local/boost_1_62_0"
}

macx {
    DEFINES += MAC_VREP
    INCLUDEPATH += "/usr/local/include"
}

unix:!macx {
    DEFINES += LIN_VREP
    INCLUDEPATH += "../../boost_1_49_0" #probably not needed when installing boost with sudo apt-get install libboost-all-dev
}

SOURCES += \
        v_repExtUrdf_a.cpp \
        urdfdialog.cpp \
	tinyxml2.cpp \
	robot.cpp \
	link.cpp \
	joint.cpp \
	sensor.cpp \
	commonFunctions.cpp \
    ../v_repMath/3Vector.cpp \
    ../v_repMath/3X3Matrix.cpp \
    ../v_repMath/4Vector.cpp \
    ../v_repMath/4X4FullMatrix.cpp \
    ../v_repMath/4X4Matrix.cpp \
    ../v_repMath/7Vector.cpp \
    ../v_repMath/MyMath.cpp \
    ../common/v_repLib.cpp \

HEADERS +=\
        v_repExtUrdf_a.h \
        urdfdialog.h \
	tinyxml2.h \
	robot.h \
	link.h \
	joint.h \
	sensor.h \
	commonFunctions.h \
    ../v_repMath/3Vector.h \
    ../v_repMath/3X3Matrix.h \
    ../v_repMath/4Vector.h \
    ../v_repMath/4X4FullMatrix.h \
    ../v_repMath/4X4Matrix.h \
    ../v_repMath/7Vector.h \
    ../v_repMath/MyMath.h \
    ../include/v_repLib.h \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}

FORMS += \
    urdfdialog.ui



















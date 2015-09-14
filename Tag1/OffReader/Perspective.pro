TEMPLATE = app
TARGET = Perspective
QT += gui opengl
CONFIG += console
HEADERS += *.h
SOURCES += Kofferraum.cpp
SOURCES += vecmath.cpp

macx: QMAKE_MAC_SDK = macosx10.9
unix:!macx: LIBS+= -lGLU

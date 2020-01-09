TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
#CONFIG -= qt

QMAKE_CXXFLAGS += -std=gnu++14

QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

DESTDIR += ./bin
OBJECTS_DIR += ./lib

HEADERS += \
    getalignresults.h \
    settings.h \
    rayint/acc/acceleration.h \
    rayint/acc/bvh_tree.h \
    rayint/acc/defines.h \
    rayint/acc/kd_tree.h \
    rayint/acc/primitives.h \
    rayint/math/algo.h \
    rayint/math/defines.h \
    rayint/math/vector.h

SOURCES += main.cpp \
    getalignresults.cpp

INCLUDEPATH += ./rayint

INCLUDEPATH += /home/eagle/CPP
LIBS += /home/eagle/CPP/libEagle_Utils.so

INCLUDEPATH += /usr/include/eigen3

LIBS += -lboost_filesystem -lboost_system

INCLUDEPATH += /usr/local/include/opencv
LIBS += -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc

INCLUDEPATH += /usr/include/ni /usr/local/cuda/include /usr/local/include/vtk-7.1 /usr/local/include/pcl-1.9
LIBS += -L"/usr/local/lib" -lpcl_common -lpcl_io -lpcl_io_ply

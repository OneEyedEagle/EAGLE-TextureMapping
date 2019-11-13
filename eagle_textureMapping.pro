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
    getkeyframes.h \
    getmeshcamera.h \
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
    getalignresults.cpp \
    getkeyframes.cpp \
    getmeshcamera.cpp

INCLUDEPATH += ./rayint ./lib
LIBS += $$PWD/lib/libEagle_Utils.so

INCLUDEPATH += /usr/local/include/eigen3

LIBS += -lboost_filesystem -lboost_system

INCLUDEPATH += /usr/local/include/opencv
LIBS += -lopencv_core -lopencv_highgui
#LIBS += -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab
LIBS += -lopencv_imgcodecs -lopencv_imgproc

INCLUDEPATH += /usr/include/ni /usr/local/cuda-10.1/include /usr/local/include/vtk-6.3 /usr/local/include/pcl-1.9
LIBS += -lpcl_gpu_kinfu_large_scale -lpcl_gpu_containers -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_visualization
LIBS += -lpcl_keypoints -lpcl_octree -lpcl_outofcore -lpcl_people -lpcl_recognition -lpcl_registration -lpcl_sample_consensus -lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_tracking


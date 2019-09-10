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
    settings.h

SOURCES += main.cpp \
    getalignresults.cpp \
    getkeyframes.cpp \
    getmeshcamera.cpp

INCLUDEPATH += /home/wsy/EAGLE/EAGLE-TextureMapping/lib
LIBS += /home/wsy/EAGLE/EAGLE-TextureMapping/lib/libEagle_Utils.so

INCLUDEPATH += /usr/local/include/eigen3

LIBS += -lboost_filesystem -lboost_system

INCLUDEPATH += /usr/local/include/opencv
LIBS += -lopencv_core -lopencv_highgui
#LIBS += -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab
LIBS += -lopencv_imgcodecs -lopencv_imgproc

INCLUDEPATH += /usr/include/ni /usr/local/cuda-10.1/include /usr/local/include/vtk-6.3 /usr/local/include/pcl-1.9
LIBS += -lpcl_gpu_kinfu_large_scale -lpcl_gpu_containers -lpcl_common -lpcl_features -lpcl_filters -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_visualization
LIBS += -lpcl_keypoints -lpcl_octree -lpcl_outofcore -lpcl_people -lpcl_recognition -lpcl_registration -lpcl_sample_consensus -lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_tracking

#LIBS += /usr/local/lib/libvtkCommonDataModel-6.3.so \
#        /usr/local/lib/libvtkCommonCore-6.3.so \
#        /usr/local/lib/libvtkCommonMath-6.3.so \
#        /usr/local/lib/libvtkRenderingLOD-6.3.so \
#        /usr/local/lib/libvtkRenderingCore-6.3.so

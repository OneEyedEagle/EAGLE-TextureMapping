#ifndef EAGLE_UTILS
#define EAGLE_UTILS

#include <string>
#include <iostream>
#include <fstream>
//#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// INCLUDEPATH += /usr/local/include/opencv
// -lopencv_core -lopencv_highgui

//#include <boost/filesystem.hpp>
// 编译时需要链接 -lboost_filesystem
// （保证能找到boost库头文件和动态路径，或者通过
// -l $(BOOST)/include/ 与 -L $(BOOST)/lib/ 添加）

namespace EAGLE{

  std::string getFilePath(std::string fullpath);
  std::string getFilename(std::string fullpath, bool withExt = true);
  bool isFileExist(std::string filename);
  bool checkPath(std::string path);
  int getFileNum(std::string, std::string);

}

#endif

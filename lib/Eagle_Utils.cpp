#include "Eagle_Utils.h"

// 获取文件所在的路径
//std::string fullpath = "/home/wsy/EAGLE/test.txt"
std::string EAGLE::getFilePath(std::string fullpath)
{
    size_t _pos = fullpath.find_last_of('/');
    std::string filename( fullpath.substr(0, _pos-1) );
    return filename;
}

// 获取路径中的文件名
//std::string fullpath = "/home/wsy/EAGLE/test.txt"
//bool withExt（默认值true）返回的文件名中包含后缀名
std::string EAGLE::getFilename(std::string fullpath, bool withExt)
{
    // fullpath = "/home/wsy/temp.txt";
    size_t _pos = fullpath.find_last_of('/');
    std::string filename( fullpath.substr(_pos + 1) );
    if (!withExt)
      filename = filename.substr(0, filename.length() - 4); // remove ext
    return filename;
}

// 检查指定文件/文件夹是否存在
//std::string filename = "/home/wsy/EAGLE/test.txt"
bool EAGLE::isFileExist(std::string filename)
{
    bool result = true;
    std::ifstream _file( filename.c_str(), std::ios::in );
    if(!_file) // it exists
        result = false;
    _file.close();
    return result;
}

// 检查指定文件夹，若不存在，则生成
//std::string path = "/home/wsy/EAGLE/Test"
bool EAGLE::checkPath(std::string path)
{
  if( isFileExist(path) ){
    return true;
  }
  system( ("mkdir " + path).c_str() );
  return true;
}

// 获取指定路径下的指定名称pattern的文件的数目（仅当前根目录）
//std::string curPath = "/home/wsy/EAGLE/EAGLE-RGSS3";
//std::string namePattern = "*.png";
int EAGLE::getFileNum(std::string curPath, std::string namePattern)
{
  std::vector<cv::String> tmp;
  cv::glob(curPath + "/" + namePattern, tmp, false);
  return tmp.size();
}

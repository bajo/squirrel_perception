#include "debugUtils.hpp"
#include "convertions.hpp"

namespace EPUtils
{

void showImageForDebug(std::string name, const cv::Mat &image, int waitTime)
{
  cv::Mat showImage;
  image.copyTo(showImage);
  
  if(showImage.type() == CV_32FC1)
  {
    cv::normalize(showImage,showImage,0,1,cv::NORM_MINMAX);
    FloatMap2UcharMap(showImage,showImage);
  }
  
  cv::imshow(name,showImage);
  cv::waitKey(waitTime);
}

void printMat(std::string name, cv::Mat &mat)
{
  std::cerr << name << ":" << std::endl;
  for(int r = 0; r < mat.rows; ++r)
  {
    for(int c = 0; c < mat.cols; ++c)
    {
      std::cerr << mat.at<float>(r,c) << " ";
    }
    std::cerr << std::endl;
  }
}

void printParameter(const int type, std::string name, float value)
{
  //debug.print(type,"[variable] %s = %4.3f\n",name.c_str(),value);
  //std::cerr << name << " = " << value << std::endl;
}

//DebugPrints::Ptr debug = DebugPrints::Ptr(new DebugPrints());

DebugPrints::DebugPrints()
{
  DebugType = LIGHT_TIME_PRINT;
  pFile = 0;
  fileOpened = false;
}

DebugPrints::~DebugPrints()
{
  if(fileOpened)
    fclose(pFile);
}

void DebugPrints::setDebugType(int type)
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  DebugType = type;
  debug_mutex.unlock();
}

void DebugPrints::openFile(const char *fileName)
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  pFile = fopen (fileName , "a");
  debug_mutex.unlock();
}

void DebugPrints::closeFile()
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  fclose(pFile);
  pFile = 0;
  debug_mutex.unlock();
}

void DebugPrints::print(const int type, const char *fmt, ...)
{
  boost::lock_guard<boost::mutex> lock(debug_mutex);
  va_list ap1;
  if (DebugType >= type)
  {
    va_start(ap1, fmt);
    if(pFile)
      vfprintf(pFile,fmt, ap1);
    else
      vprintf(fmt, ap1);
    va_end(ap1);
  }
  debug_mutex.unlock();
}

//DebugPrints::Ptr DebugPrints::getDebug()
//{
  //if(debug == NULL)
  //{
    //DebugPrints::Ptr debug = DebugPrints::Ptr(new DebugPrints());
  //}
  //return debug;
//}

} //namespace EPUtils
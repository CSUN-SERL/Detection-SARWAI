#include "local_image_logging_strategy.h"
#include "logging_strategy_registry.h"
#include "ros/ros.h"
#include <fstream>
#include <sys/stat.h>
#include <errno.h> //debugging
#include <stdio.h> //debugging
#include <sstream>
#include <ctime>
#include <dirent.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace sarwai {
  
  void LocalImageLoggingStrategy::Log(sensor_msgs::Image& image, BoxMetadata boxdata) {
    /**/
    int filenum = 1;
    std::stringstream imagepath;
    std::stringstream textpath;
    time_t currtime = time(0);
    struct tm * date = localtime( &currtime );
    textpath << (date->tm_year + 1900) << "/" << (date->tm_mon + 1) << "/" << date->tm_mday << "/";;
    imagepath << textpath.str();
    std::ofstream textout;

    DIR* dir = opendir(textpath.str().c_str());
    if(dir) {
      struct dirent * ent;
      closedir(dir);

      textpath << "text/";
      imagepath << "image/";
      textpath << "output.txt";

      dir = opendir(imagepath.str().c_str());
      while((ent = readdir(dir)) != NULL) {
        char* filename = ent->d_name;
        if(!IsImageFile(filename)) {
          continue;
        }
        int num = GetFileNumber(ent->d_name);
        if(filenum < num) {
          filenum = num;
        }
      }
      ++filenum;
      imagepath << "image_" << filenum << ".jpg";
      closedir(dir);
    }

    else if (ENOENT == errno) {
      textpath << "text/output.txt";
      imagepath << "image/";
      /**/
      std::stringstream textpathmaker;
      textpathmaker << (date->tm_year + 1900) << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << (date->tm_mon + 1) << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << date->tm_mday << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << "text/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      mkdir(imagepath.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      imagepath << "image_1.jpg";
      closedir(dir);
      /**/

    }
    else {
      ROS_ERROR("Unable to open directory");
      return;
    }

    textout.open(textpath.str(), std::ofstream::app);
    if(!(textout.is_open())){
      ROS_INFO("textout aint open");
    }

    // output image
    cv_bridge::CvImagePtr img;

    try {
      img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("CV_Bridge Excepion: %s", e.what());
      return;
    }

    cv::Mat mat = img->image;
    cv::imwrite(imagepath.str(), mat);

    // output text
    std::ostringstream formattedstring;
    formattedstring << "class:" << boxdata.object_class << ";confidence:" << boxdata.confidence_rating << ";timestamp:" << boxdata.timestamp << ";x-coord:" << boxdata.left_x_coord << ";y-coord:" << boxdata.top_y_coord << ";width:" << boxdata.box_width << ";height:" << boxdata.box_height << ";depth:" << boxdata.depth <<"m;imagefilename:" << "image_" << filenum << ".jpg,\n";
    textout << formattedstring.str();
    /**/
    
  }

  int LocalImageLoggingStrategy::GetFileNumber(const char * filename) const {
        unsigned index = 0;
        while(filename[index] != '\0') {
          index++;
        }
        int ret = 0;
        int mult = 1;
        index -= 5; // ".jpg" is 4 characters, 5 back is the first filenum character
        do{
          ret += (int)(filename[index--] - '0') * mult;
          mult *= 10;
        } while (isdigit(filename[index]));
        return ret;
  }

  bool LocalImageLoggingStrategy::IsImageFile(const char * filename) const {
    // Absolutely gross implementation, but we'll probably make it better later.
    if(filename[0] == '.') {
      return false;
    }
    return true;
  }
  
  REGISTER_STRATEGY(LocalImageLoggingStrategy)
}

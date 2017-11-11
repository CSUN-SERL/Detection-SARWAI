#include "local_audio_logging_strategy.h"

#include <ros/ros.h>
#include <fstream>
#include <sys/stat.h>
#include <errno.h> //debugging
#include <stdio.h> //debugging
#include <sstream>
#include <ctime>
#include <dirent.h>

namespace sarwai {
  
  void LocalAudioLoggingStrategy::Log(std::string audio_file_name, struct AudioMetadata metadata) {
    std::stringstream textpath;
    time_t currtime = time(0);
    struct tm* date = localtime(&currtime);
    textpath << (date->tm_year + 1900) << "/" << (date->tm_mon + 1) << "/" << date->tm_mday << "/";
    
    std::ofstream textout;

    DIR* dir = opendir(textpath.str().c_str());
    if(dir) {
      struct dirent * ent;
      closedir(dir);

      textpath << "text/output.txt";
    }

    else if (ENOENT == errno) {
      textpath << "text/output.txt";
      std::stringstream textpathmaker;
      textpathmaker << (date->tm_year + 1900) << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << (date->tm_mon + 1) << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << date->tm_mday << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << "text/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      closedir(dir);

    }
    else {
      ROS_ERROR("Unable to open directory");
      return;
    }

    textout.open(textpath.str(), std::ofstream::app); 
    if(!(textout.is_open())){
      std::stringstream textpathmaker;
      textpathmaker << (date->tm_year + 1900) << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << (date->tm_mon + 1) << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << date->tm_mday << "/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      textpathmaker << "text/";
      mkdir(textpathmaker.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      if(!textout.is_open()) {
        ROS_INFO("textout aint open");
      }
    }

    std::ostringstream formattedstring;
    formattedstring <<
    "audio," <<
    metadata.confidence << "," <<
    audio_file_name <<
    std::endl;
    textout << formattedstring.str();
  }
  //REGISTER_STRATEGY(LocalImageLoggingStrategy)
}
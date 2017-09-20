#include "local_logging_strategy.h"
#include "logging_strategy_registry.h"
#include <fstream>
#include <sys/stat.h>
#include <sstream>
#include <ctime>
#include <dirent.h>

namespace sarwai {
  
  void LocalLoggingStrategy::Log(std::vector<int8_t> image, BoxMetadata boxdata) {
    int filenum = 1;
    std::stringstream imagepath;
    std::stringstream textpath;
    time_t currtime = time(0);
    struct tm * date = localtime( &currtime );
    textpath << "/" << (date->tm_year + 1900) << "/" << (date->tm_mon + 1) << "/" << date->tm_mday;
    imagepath << textpath.str();
    // imagepath << "/image/";

    struct stat info;
    if(stat( textpath.str().c_str(), &info) != 0) {
      // TODO: Error handling for unaccessible folder
    }
    else if(!(info.st_mode & S_IFDIR)) {
      // in an unmade directory
      textpath << "/text/output_1.txt";
      imagepath << "/image/image_1.png";
      mkdir(textpath.str().c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
      mkdir(imagepath.str().c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
    }
    else {
      // in a made directory
      DIR* dir;
      struct dirent * ent;
      textpath << "/text/";
      imagepath << "/image/";
      dir = opendir(textpath.str().c_str()); // Assuming this will work because stat found something.
      while((ent = readdir(dir)) != NULL) {
        int num = GetFileNumber(ent->d_name);
        if(filenum < num) {
          filenum = num;
        }
      }
      ++filenum;
      textpath << "/text/output_" << filenum << ".txt";
      closedir(dir);
      mkdir(textpath.str().c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
      
      filenum = 1;
      dir = opendir(imagepath.str().c_str());
      while((ent = readdir(dir)) != NULL) {
        int num = GetFileNumber(ent->d_name);
        if(filenum < num) {
          filenum = num;
        }
      }
      ++filenum;
      imagepath << "/image/image_" << filenum << ".png";
      closedir(dir);
      mkdir(imagepath.str().c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
    }

    std::ofstream textout(textpath.str(), std::ofstream::app);
    std::ofstream imageout(imagepath.str(), std::ofstream::out);
    if(!(textout.is_open() && imageout.is_open())){
      // TODO: Error handling
    }

    // output image

    for(unsigned i = 0; i < image.size(); ++i) {
      imageout << image[i];
    }

    // output text
    std::ostringstream formattedstring;
    formattedstring << "class:" << boxdata.object_class << ";confidence:" << boxdata.confidence_rating << ";timestamp:" << boxdata.timestamp << ";x-coord:" << boxdata.left_x_coord << ";y-coord:" << boxdata.top_y_coord << ";width:" << boxdata.box_width << ";height:" << boxdata.box_height << "imagefilename:" << "image_" << filenum << ".png,";
    textout << formattedstring.str();
    
  }

  int LocalLoggingStrategy::GetFileNumber(const char * filename) const {
        unsigned index = 0;
        while(filename[index] != '\0') {
          index++;
        }
        return index - 5; // ".txt" is 4 characters, 5 back is number
  }
  
  REGISTER_STRATEGY(LocalLoggingStrategy)
}

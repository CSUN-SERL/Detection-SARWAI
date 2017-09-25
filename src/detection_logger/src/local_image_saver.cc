#include "local_image_saver.h"

#include <fstream>
#include <sys/stat.h>
#include <sstream>
#include <ctime>
#include <dirent.h>

namespace sarwai {
    std::string LocalImageSaver::SaveImage(std::vector<uint8_t> image) {
        int filenum = 1;
        std::stringstream imagepath;
        time_t currtime = time(0);
        struct tm * date = localtime(&currtime);
        imagepath << "/" << (date->tm_year + 1900) << "/" << (date->tm_mon + 1) << "/" << date->tm_mday << "/image/";

        struct stat info;
        if(stat( textpath.str().c_str(), &info) != 0) {
            // TODO: Error handling for unaccessible folder
        }
        else if(!(info.st_mode & S_IFDIR)) {
            // In an unmade directory
            imagepath << "/image_1.png";
            mkdir(imagepath.str().c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
        }
        else {
            // In a made directory
            DIR* dir;
            struct dirent * ent;

            dir = opendir(imagepath.str().c_str());
            while((ent = readdir(dir)) != NULL) {
                int num = GetFileNumber(ent->d_name);
                if(filenum < num) {
                    filenum = num;
                }
            }
            ++filenum;
            imagepath << "/image_" << filenum << ".png";
            closedir(dir);
            mkdir(imagepath.str().c_str(), S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
        }

        
    }
}
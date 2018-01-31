#include <string>
#include <ctime>
#include <iostream>

#include "boost/filesystem.hpp"

#include "logger.h"

namespace sarwai {

  Logger::Logger(std::string base_filepath, std::string mission_id) {
    std::string full_dir_path = base_filepath + boost::filesystem::path::preferred_separator + "log-" + mission_id;
    CreateLogDirectory(full_dir_path);
  }

  Logger::Logger(std::string base_filepath) {

    std::time_t timestamp = std::time(0);
    std::string full_dir_path = base_filepath + boost::filesystem::path::preferred_separator + "log-" + std::to_string((int)timestamp);
    CreateLogDirectory(full_dir_path);
  }

  Logger::Logger() {}

  void Logger::CreateLogDirectory(std::string full_dir_path) {
    boost::filesystem::path dir(full_dir_path);
    if (boost::filesystem::create_directory(dir)) {
      std::cout << "Created log directory: " << full_dir_path << "\n";
    } 
    else {
      std::cout << full_dir_path << " already exists! Uh oh!" << "\n";
    }
  }
}
#ifndef SARWAI_DETEECTION_LOGGER_LOGGER_H_
#define SARWAI_DETEECTION_LOGGER_LOGGER_H_

#include <string>

namespace sarwai {
  class Logger {
    public:
    Logger(std::string base_filepath, std::string mission_id);
    Logger(std::string base_filepath);
    // To be used when logging is done remotely
    Logger();

    protected:
    std::string log_filepath_;
    // Identifying suffix string to append to the log directory name.
    std::string mission_id_;

    void CreateLogDirectory(std::string full_dir_path);

    private:
  };
}

#endif
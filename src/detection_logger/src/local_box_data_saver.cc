#include "local_box_data_saver.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>
// #include <ostringstream>
#include <sstream>

namespace sarwai{

  LocalBoxDataSaver::LocalBoxDataSaver() {
    // empty
  }
  
  LocalBoxDataSaver::~LocalBoxDataSaver() {
    // empty
  }

  void LocalBoxDataSaver::SaveBoxData(const BoxMetadata & boxdata, std::string filename, std::string filepath) {
    std::ofstream out((filepath + "/" + filename).c_str(), std::ofstream::app);
    
    if(!out.is_open()) {
      mkdir((filepath + "/" + filename).c_str(), S_IFDIR);
      out.open((filepath+"/"+filename).c_str(), std::ofstream::app);
      if(!out.is_open()) {
        // TODO: error handling
      }
    }

    std::string formatted_output = FormatOutput(boxdata);
    
    // WARNING: we MIGHT need to make the buffer "formattedOutput.length() + 1"
    // if it doesn't automatically concatenate the null terminator
    out.write(formatted_output.c_str(), formatted_output.length());
  }
  
  std::string LocalBoxDataSaver::FormatOutput(const BoxMetadata & boxdata) {
    std::ostringstream formattedstring;
    formattedstring << "class:" << boxdata.object_class << ";confidence:" << boxdata.confidence_rating << ";timestamp:" << boxdata.timestamp << ";x-coord:" << boxdata.left_x_coord << ";y-coord:" << boxdata.top_y_coord << ";width:" << boxdata.box_width << ";height:" << boxdata.box_height << ",";
    
    return formattedstring.str();
  }
}

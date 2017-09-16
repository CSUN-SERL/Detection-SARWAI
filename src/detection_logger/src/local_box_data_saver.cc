#include "local_box_data_saver.h"
#include "sys/stat.h"
#include <fstream>
#include <ostringstream>

namespace sarwai{

  LocalBoxDataSaver::LocalBoxDataSaver() {
    // empty
  }
  
  LocalBoxDataSaver::~LocalBoxDataSaver() {
    // empty
  }

  void LocalBoxDataSaver::SaveBoxData(const BoxMetadata & boxdata, std::string filename, std::string filepath) {
    ofstream out((filepath + "/" + filename).c_str(), std::ofstream::app);
    
    if(!out.is_open()) {
      mkdir((filepath + "/" + filename).c_str());
      out.open((filepath+"/"+filename).c_str(), std::ofstream::app);
      if(!out.is_open()) {
        // TODO: error handling
      }
    }

    std::string formattedoutput = FormatOutput(boxdata);
    
    // WARNING: we MIGHT need to make the buffer "formattedOutput.length() + 1"
    // if it doesn't automatically concatenate the null terminator
    out.write(formattedOutput.c_str(), formattedOutput.length());
  }
  
  std::string LocalBoxDataSaver::FormatOutput(const BoxMetadata & boxdata) {
    std::ostringstream formattedstring;
    formattedstring << "class:" << boxdata.object_class << ";confidence:" << boxdata.confidence_rating << ";timestamp:" << boxdata.timestamp << ";x-coord:" << boxdata.left_x_coord << ";y-coord:" << boxdata.top_y_coord << ";width:" << boxdata.box_width << ";height:" << boxdata.box_height << ",";
    
    return formattedstring.str();
  }
}

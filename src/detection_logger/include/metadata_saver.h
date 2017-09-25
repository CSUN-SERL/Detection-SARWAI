#ifndef SARWAI_DETECTION_LOGGER_METADATA_SAVER_H_
#define SARWAI_DETECTION_LOGGER_METADATA_SAVER_H_

#include <string>
#include "box_metadata.h"

namespace sarwai {

    class MetadataSaver {
    public:
        void SaveMetadata(BoxMetadata metadata, std::string imagefilename) = 0;
    private:

    };

}

#endif
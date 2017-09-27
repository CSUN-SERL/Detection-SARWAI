#ifndef SARWAI_DETECTION_LOGGER_METADATA_SAVER_H_
#define SARWAI_DETECTION_LOGGER_METADATA_SAVER_H_

#include <string>
#include "box_metadata.h"

namespace sarwai {

    class MetadataSaver {
    public:

        /**
        *   Save the metadata of bounding boxes to the user's local file system.
        *   @param metadata BoxMetadata object containing the bounding box information.
        *   @param imagefilename String containing the file name of the associated image on disk.
        */
        void SaveMetadata(BoxMetadata metadata, std::string imagefilename) = 0;
    private:

    };

}

#endif
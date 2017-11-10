#ifndef SARWAI_DETECTION_LOGGER_AUDIO_METADATA_H_
#define SARWAI_DETECTION_LOGGER_AUDIO_METADATA_H_

namespace sarwai {

    struct AudioMetadata {
        int start_timestamp;
        float confidence;
    };

}

#endif
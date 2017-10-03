#ifndef SARWAI_DETECTION_LOGGER_AUDIO_LOGGING_STRATEGY_H_
#define SARWAI_DETECTION_LOGGER_AUDIO_LOGGING_STRATEGY_H_



namespace sarwai {

    #define INIT_AUDIO_STRATEGY(STRATEGYNAME) public: static std::string Classname() {return #STRATEGYNAME;} static AudioLoggingStrategy * Get(){return new STRATEGYNAME;}

    class AudioLoggingStrategy {
    public:
        // virtual void Log(sensor_msgs::Image& image, BoxMetadata boxdata) = 0;
    };
}

#endif
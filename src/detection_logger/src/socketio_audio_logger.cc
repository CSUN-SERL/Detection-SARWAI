#include "sio_client.h"
#include "sio_message.h"
#include "sio_socket.h"

#include "socketio_audio_logger.h"
#include "audio_detection_data.h"

namespace sarwai {
  
  SocketIOAudioLogger::SocketIOAudioLogger(std::string host_addr, int port) {
    host_addr_ = host_addr;
    port_ = port;
  }

  void SocketIOAudioLogger::Log(struct AudioDetectionData data) {
    SendData(data);
  }

  void SocketIOAudioLogger::SendData(struct AudioDetectionData data) {
    std::string connection_string = host_addr_ + ":" + std::to_string(port_);
    sio::client socket_client;
    socket_client.connect(connection_string);

    std::string csv_data = GenerateStringCSV(data);
    socket_client.socket("/socket.io")->emit(audio_detection_event_name_, csv_data);
  }
}
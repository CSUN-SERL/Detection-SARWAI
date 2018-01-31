
#include "sio_client.h"
#include "sio_message.h"
#include "sio_socket.h"

#include "socketio_visual_logger.h"

namespace sarwai {
  
  SocketIOVisualLogger::SocketIOVisualLogger(std::string base_filepath,
      std::string host_addr,
      int port) : VisualLogger(base_filepath) {

    host_addr_ = host_addr;
    port_ = port;
  }

  void SocketIOVisualLogger::Log(cv::Mat image, struct VisualDetectionData data) {
    std::string image_filename = SaveImage(image);
    SendData(data, image_filename);
  }

  void SocketIOVisualLogger::Log(std::string image_filename, struct VisualDetectionData data) {
    SendData(data, image_filename);
  }

  void SocketIOVisualLogger::SendData(struct VisualDetectionData data, std::string image_filename) {
    std::string connection_string = host_addr_ + ":" + std::to_string(port_);
    sio::client socket_client;
    socket_client.connect(connection_string);

    std::string csv_data = GenerateStringCSV(data, image_filename);

    socket_client.socket()->emit(visual_detection_event_name_, csv_data);
  }
}
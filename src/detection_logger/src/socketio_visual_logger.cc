#include <string>
#include <sstream>
#include <stdlib.h>

#include "socketio_visual_logger.h"

namespace sarwai {
  
  SocketIOVisualLogger::SocketIOVisualLogger(std::string base_filepath,
      std::string host_addr,
      int port) {

    host_addr_ = host_addr;
    port_ = port;

    visual_detection_event_name_ = "detection-insert-query";

    std::string connection_string = host_addr_ + ":" + std::to_string(port_);
    std::cout << connection_string << "\n";
    socket_client_.connect(connection_string);
    socket_client_.socket("/socket.io")->on("query-id",
        std::bind( &SocketIOVisualLogger::ReceiveQueryId, this, std::placeholders::_1));
  }

  void SocketIOVisualLogger::Log(cv::Mat image, struct VisualDetectionData data) {
    std::string image_filename = SaveImage(image);
    SendData(data, image_filename);
  }

  void SocketIOVisualLogger::Log(std::string image_filename, struct VisualDetectionData data) {
    SendData(data, image_filename);
  }

  void SocketIOVisualLogger::ReceiveQueryId(sio::event &queryIdEvent) {
    int query_id = queryIdEvent.get_message()->get_int();
    std::cout << "received query id: " << query_id << "\n";
    std::string iris_script_cmd = "python ~/programming/sarwai/adaptation/clustering/evaluate.py " + query_id;
    system(iris_script_cmd.c_str());
  }

  void SocketIOVisualLogger::SendData(struct VisualDetectionData data, std::string image_filename) {
    std::string json_data = GenerateJSONString(data, image_filename);
    std::string csv_data = GenerateStringCSV(data, image_filename);
    std::cout << "JSON: " << json_data << "\n";
    socket_client_.socket("/socket.io")->emit(visual_detection_event_name_, json_data);
  }

  std::string SocketIOVisualLogger::GenerateJSONString(struct VisualDetectionData data, std::string image_filename) {
    std::stringstream json;
    json << "{"
    <<"\""<<"type"<<"\":" << "\"" << "visual" << "\"" << "," 
    <<"\""<<"timestamp"<<"\":" << "\"" << data.timestamp << "\"" << "," 
    <<"\""<<"robotId"<<"\":" << "\"" << data.robot_id << "\"" << "," 
    <<"\""<<"confidence"<<"\":" << "\"" << data.confidence_rating << "\"" << "," 
    <<"\""<<"filePath"<<"\":" << "\"" << image_filename << "\"" 
    <<"}";

    return json.str();
  }
}
roscore &
clear
sleep 5
source /opt/ros/kinetic/setup.bash
roslaunch video_stream_opencv webcam.launch camera_name:=webcam1 video_stream_provider:=0 &
clear
sleep 2
roslaunch video_stream_opencv webcam.launch camera_name:=webcam2 video_stream_provider:=1 &
clear
sleep 2
roslaunch video_stream_opencv webcam.launch camera_name:=webcam3 video_stream_provider:=2 &
clear
sleep 2
roslaunch video_stream_opencv webcam.launch camera_name:=webcam4 video_stream_provider:=3 &
clear
sleep 2
clear
cd Detection-SARWAI
source devel/setup.bash
roslaunch darknet_ros darknet_ros.launch &
sleep 5
clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg name:=track1 &
sleep 3
clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg2 name:=track2 &
sleep 3
clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg3 name:=track3 &
sleep 3
clear
roslaunch tracker tracker.launch topic_name_:=/detection/compiled_ros_msg4 name:=track4 &
sleep 3
clear
rosrun image_draw image_draw_node &
sleep 3
clear
rosrun detection_logger visual_logger &



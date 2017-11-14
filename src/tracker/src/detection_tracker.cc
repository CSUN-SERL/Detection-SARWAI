#include "detection_tracker.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


namespace sarwai {

VisualDetectionTracker::VisualDetectionTracker() 
{
	bool isEmpty = true;
    this->nh_ = new ros::NodeHandle();
    this->image_frame_sub_ = this->nh_->subscribe(
      "darknet_ros/detection_image", 1000, &VisualDetectionTracker::ImageCallback, this);
    //subscribes to darknet_ros/bounding_boxes    
    this->bounding_box_sub_ = this->nh_->subscribe(
      "darknet_ros/bounding_boxes", 1000, &VisualDetectionTracker::ArrayReceived, this); 
    //subscribes to darknet_ros/found_object
    this->detection_flag_sub_ = this->nh_->subscribe(
      "darknet_ros/found_object", 1000, &VisualDetectionTracker::ObjectDetected, this); 

  	this->visual_detection_image_ = this->nh_->advertise<sensor_msgs::Image>(
        "visual_detection_image", 1000);

	this->visual_detection_bb_ = this->nh_->advertise<darknet_ros_msgs::BoundingBoxes>(
        "visual_detection_bb", 1000); 

	this->tracking_algorithm_ = TrackingAlgorithm::BOOSTING;
}

VisualDetectionTracker::~VisualDetectionTracker() 
{

}


void VisualDetectionTracker::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	this->video_image_frames_.push(*msg);
	process();
}


void VisualDetectionTracker::ArrayReceived(const darknet_ros_msgs::BoundingBoxes& msg)
{
	this->bounding_boxes_matrix_.push(msg.boundingBoxes); 
}


void VisualDetectionTracker::ObjectDetected(const std_msgs::Int8& msg)
{
	this->detection_flag_.push(msg.data);
}


void VisualDetectionTracker::process()
{
	bounding_boxes = bounding_boxes_matrix_.front();
	std::vector<cv::Rect2d> detection_bbs;

	for(int i=0; i<bounding_boxes.size(); i++)
	{
		darknet_ros_msgs::BoundingBox bb = bounding_boxes.at(i);
		cv::Rect2d bb_rect(bb.xmin, bb.ymin, bb.xmax - bb.xmin, bb.ymax - bb.ymin);
		detection_bbs.push_back(bb_rect);
	}
	sensor_msgs::Image master_image = this->video_image_frames_.front();

	TrackFrame(cv_bridge::toCvCopy(master_image, sensor_msgs::image_encodings::BGR8)->image);

	if(isEmpty)
	{
		AddTrackers(cv_bridge::toCvCopy(master_image, sensor_msgs::image_encodings::BGR8)->image, detection_bbs);
		//this->visual_detection_bb_.publish();
		this->visual_detection_image_.publish(master_image);
	}

	
	video_image_frames_.pop();
	bounding_boxes_matrix_.pop();

}


  bool VisualDetectionTracker::IsRedundantDetection(cv::Rect2d detection_bb, std::vector<cv::Rect2d> tracking_bbs) {
    float det_bb_area = detection_bb.width * detection_bb.height;

    for (int i = 0; i < tracking_bbs.size(); i++) {
      cv::Rect2d tracking_bb = tracking_bbs.at(i);
      // Compare detection_bb to all bounding boxes in tracking_bbs

      // AREA DIFFERENCE
      //float area_difference_percentage = 0.0;
      float tracking_bb_area = tracking_bb.width * tracking_bb.height;

      if (std::min(det_bb_area, tracking_bb_area) / std::max(det_bb_area, tracking_bb_area)  > 0.5) {
        return false;
      }

    }

    return true;
  }


  void VisualDetectionTracker::TrackFrame(const cv::Mat &image_matrix) {
  	if(this->trackers_.size() == 0)
  	{
  		isEmpty = true;
  		return;
  	}

    for (int i = 0; i < this->trackers_.size(); i++) 
    {
      cv::Mat image_copy = image_matrix.clone();
      cv::Ptr<cv::Tracker> tracker = this->trackers_.at(i);
      cv::Rect2d bb = this->tracking_boxes_.at(i);
      bool object_tracked = tracker->update(image_copy, bb);

      if (object_tracked) 
      {
      	tracking_boxes_.at(i) = bb;
        cv::Point top_left = cv::Point(bb.x, bb.y);
        cv::Point bottom_right = cv::Point(bb.x + bb.width,
          bb.y + bb.height);
          ROS_INFO("%f, %f", bb.x, bb.y);
          cv::rectangle(image_copy, bb, 16711808, 10, 143);
          cv::imshow("tracking", image_copy);
          cv::waitKey(1);
          isEmpty = false;
      }
      else 
      {
        this->trackers_.erase(this->trackers_.begin() + i);
        this->tracking_boxes_.erase(this->tracking_boxes_.begin() + i);
        if (this->trackers_.size() == 0) 
      		break;
      }
    }

  }


  void VisualDetectionTracker::AddTrackers(const cv::Mat &image, std::vector<cv::Rect2d> detection_bbs) {
  	if (HasActiveTrackers()) {
  		return;
  	}

    for (int i = 0; i < detection_bbs.size(); i++) {
    	cv::Rect2d det_bb = detection_bbs.at(i);
		cv::Ptr<cv::Tracker> new_tracker;
		switch (this->tracking_algorithm_) {
			case TrackingAlgorithm::BOOSTING :
				new_tracker = cv::TrackerBoosting::create();
				break;
			case TrackingAlgorithm::MIL :
				new_tracker = cv::TrackerMIL::create();
				break;
			case TrackingAlgorithm::KCF :
				new_tracker = cv::TrackerKCF::create(); 
				break;
			case TrackingAlgorithm::TLD :
				new_tracker = cv::TrackerTLD::create(); 
				break;
			case TrackingAlgorithm::MEDIANFLOW :
				new_tracker = cv::TrackerMedianFlow::create(); 
				break;
			case TrackingAlgorithm::GOTURN : 
				new_tracker = cv::TrackerGOTURN::create(); 
				break;
			default:
				return;
		}

        new_tracker->init(image, det_bb);
        this->trackers_.push_back(new_tracker);
        this->tracking_boxes_.push_back(det_bb);
    }

  }


  bool VisualDetectionTracker::HasActiveTrackers() {
    bool hasActive = this->trackers_.size() != 0;
    return hasActive;
  }

}

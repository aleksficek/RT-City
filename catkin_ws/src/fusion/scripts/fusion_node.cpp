#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<jsk_recognition_msgs::BoundingBoxArray>("published_topic", 1);
    //Topic you want to subscribe
    sub_ = n_.subscribe("bbox_array", 1, &SubscribeAndPublish::callback, this);
  }

  // jsk_recognition_msgs::BoundingBoxArray

  void callback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& some_msg)
  {
    ROS_INFO("Received bbox");
    pub_.publish(some_msg);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
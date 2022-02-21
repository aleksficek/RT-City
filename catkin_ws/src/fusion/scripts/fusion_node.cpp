#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::String>("published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const std_msgs::String::ConstPtr& some_msg)
  {
    // std_msgs::String output;
    //.... do something with the input and generate the output...
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << 4;
    msg.data = ss.str();
    pub_.publish(msg);
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
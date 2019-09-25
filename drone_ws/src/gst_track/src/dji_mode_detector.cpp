#include <ros/ros.h>
#include <iterator>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>


/*
Doesn't work yet: when catkin_make, can't find new node?
*/
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Int32>("/mode", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/dji_sdk/rc", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::Joy& input)

  {
    std_msgs::Int32 mode;

    if(input.axes[4] == -8000.0)
    {
      // P mode
      mode.data = 0;
    }
    else if(input.axes[4] == 0.0)
    {
      // A mode
      mode.data = 1;
    }
    else if(input.axes[4] == 8000.0)
    {
      // F mode
      mode.data = 2;
    }
    else
    {
      // This state should never happen
      mode.data = 3;
    }
    //.... do something with the input and generate the output...
    pub_.publish(mode);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "dji_mode_detector");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

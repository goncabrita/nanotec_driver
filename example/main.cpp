#include "ros/ros.h"
#include "nanotec_driver/NanotecMotor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nanotec_example");

  ros::NodeHandle n;

  ROS_INFO("Nanotec for ROS Example");

  std::string port_name = "/dev/ttyACM1";

  nanotec::NanotecPort port;
  port.openPort(port_name, 115200);

  nanotec::NanotecMotor motor(&port);

  try{ motor.setPositionMode(NANOTEC_POSITION_MODE_RELATIVE); }
  catch(nanotec::Exception& e)
  {
      ROS_FATAL("%s", e.what());
      ROS_BREAK();
  }

  try{ motor.setDirection(NANOTEC_DIRECTION_LEFT); }
  catch(nanotec::Exception& e)
  {
      ROS_FATAL("%s", e.what());
      ROS_BREAK();
  }

  try{ motor.setTravelDistance(5000); }
  catch(nanotec::Exception& e)
  {
      ROS_FATAL("%s", e.what());
      ROS_BREAK();
  }

  try{ motor.setMinimumFrequency(200); }
  catch(nanotec::Exception& e)
  {
      ROS_FATAL("%s", e.what());
      ROS_BREAK();
  }

  try{ motor.setMaximumFrequency(2000); }
  catch(nanotec::Exception& e)
  {
      ROS_FATAL("%s", e.what());
      ROS_BREAK();
  }

  try{ motor.setAccelerationRamp(1000); }
  catch(nanotec::Exception& e)
  {
      ROS_FATAL("%s", e.what());
      ROS_BREAK();
  }

  try{ motor.startMotor(); }
  catch(nanotec::Exception& e)
  {
      ROS_FATAL("%s", e.what());
      ROS_BREAK();
  }

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}

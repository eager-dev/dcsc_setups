using namespace std;

#include <libusb-1.0/libusb.h>
#include "dcsc_setups/PendulumWrite.h"
#include "dcsc_setups/PendulumRead.h"
#include "dcsc_setups/usb_utils.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>
#include <math.h>
#include <signal.h>

FUGIPendulum pendulum(0);


bool init()
{
  int status;
  if ((status = pendulum.write()) != EOK) {
    ROS_ERROR("[pendulum] Writing failed with status [%s], I/O error [%s]", strerror(status), strerror(pendulum.write_error()));
    return false;
  }
  pendulum.actuators.digital_outputs = 1;
  pendulum.actuators.voltage0 = 0.0;
  pendulum.actuators.voltage1 = 0.0;
  pendulum.actuators.timeout = 1.0;
  return true;
}


bool write(dcsc_setups::PendulumWrite::Request &req, dcsc_setups::PendulumWrite::Response &res)
{
  int status;
  pendulum.actuators.digital_outputs = req.actuators.digital_outputs;
  pendulum.actuators.voltage0 = req.actuators.voltage0;
  pendulum.actuators.voltage1 = req.actuators.voltage1;
  pendulum.actuators.timeout = req.actuators.timeout;
  if ((status = pendulum.write()) != EOK) {
    res.success = false;
    res.message = ("Writing failed with status [%s], I/O error [%s]", strerror(status), strerror(pendulum.write_error()));
    return true;
  }
  res.success = true;
  return true;
}


bool read(dcsc_setups::PendulumRead::Request &req, dcsc_setups::PendulumRead::Response &res)
{
  int status;
  if ((status = pendulum.read()) != EOK) {
    ROS_ERROR("[pendulum] Reading failed with status [%s], I/O error [%s]", strerror(status), strerror(pendulum.read_error()));
    res.success = false;
    res.message = ("Reading failed with status [%s], I/O error [%s]", strerror(status), strerror(pendulum.read_error()));
    return true;
  }
  res.sensors.header.stamp = ros::Time::now();
  res.sensors.relative_time = pendulum.sensors.relative_time;
  res.sensors.current = pendulum.sensors.current;  
  res.sensors.position0 = pendulum.sensors.position0;
  res.sensors.position1 = pendulum.sensors.position1;
  res.sensors.voltage_beam = pendulum.sensors.voltage_beam;
  res.sensors.voltage_pendulum = pendulum.sensors.voltage_pendulum;
  res.sensors.digital_inputs = pendulum.sensors.digital_inputs;
  return true;
}

void mySigintHandler(int sig)
{
      // Do some custom action.
      // For example, publish a stop message to some other nodes.

      // All the default sigint handler does is call shutdown()
     ros::shutdown();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pendulum", ros::init_options::NoSigintHandler);
  ros::NodeHandle n("~");

  // Override the default ros sigint handler.
  signal(SIGINT, mySigintHandler);

  bool initialized = false;
  unsigned int count = 0;

  while (!initialized)
  {
    ++ count;
    if (count > 5)
    {
      ROS_ERROR("[pendulum] Maximum number of attempts to initialize Pendulum reached.");
      return 0;
    }
    initialized = init();
  }

  ros::ServiceServer write_service = n.advertiseService("write", write);
  ros::ServiceServer read_service = n.advertiseService("read", read);

  ros::spin();

  return 0;
}
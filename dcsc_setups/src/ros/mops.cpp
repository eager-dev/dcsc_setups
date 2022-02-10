using namespace std;

#include <libusb-1.0/libusb.h>
#include "dcsc_setups/MopsWrite.h"
#include "dcsc_setups/MopsRead.h"
#include "dcsc_setups/usb_utils.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>
#include <math.h>
#include <signal.h>

FUGIMops mops(0);


bool init()
{
  int status;
  if ((status = mops.write()) != EOK) {
    ROS_ERROR("[mops] Writing failed with status [%s], I/O error [%s]", strerror(status), strerror(mops.write_error()));
    return false;
  }
  mops.actuators.digital_outputs = 1;
  mops.actuators.voltage0 = 0.0;
  mops.actuators.voltage1 = 0.0;
  mops.actuators.timeout = 1.0;
  return true;
}


bool write(dcsc_setups::MopsWrite::Request &req, dcsc_setups::MopsWrite::Response &res)
{
  int status;
  mops.actuators.digital_outputs = req.actuators.digital_outputs;
  mops.actuators.voltage0 = req.actuators.voltage0;
  mops.actuators.voltage1 = req.actuators.voltage1;
  mops.actuators.timeout = req.actuators.timeout;
  if ((status = mops.write()) != EOK) {
    res.success = false;
    res.message = ("Writing failed with status [%s], I/O error [%s]", strerror(status), strerror(mops.write_error()));
    return true;
  }
  res.success = true;
  return true;
}


bool read(dcsc_setups::MopsRead::Request &req, dcsc_setups::MopsRead::Response &res)
{
  int status;
  if ((status = mops.read()) != EOK) {
    ROS_ERROR("[mops] Reading failed with status [%s], I/O error [%s]", strerror(status), strerror(mops.read_error()));
    res.success = false;
    res.message = ("Reading failed with status [%s], I/O error [%s]", strerror(status), strerror(mops.read_error()));
    return true;
  }
  res.sensors.header.stamp = ros::Time::now();
  res.sensors.position0 = mops.sensors.position0;
  res.sensors.position1 = mops.sensors.position1;
  res.sensors.speed = mops.sensors.speed;
  res.sensors.voltage = mops.sensors.voltage;
  res.sensors.current = mops.sensors.current;
  res.sensors.digital_inputs = mops.sensors.digital_inputs;
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
  ros::init(argc, argv, "mops", ros::init_options::NoSigintHandler);
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
      ROS_ERROR("[mops] Maximum number of attempts to initialize MOPS reached.");
      return 0;
    }
    initialized = init();
  }

  ros::ServiceServer write_service = n.advertiseService("write", write);
  ros::ServiceServer read_service = n.advertiseService("read", read);

  ros::spin();

  return 0;
}

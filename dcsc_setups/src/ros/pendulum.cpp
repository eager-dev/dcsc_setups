using namespace std;

#include <libusb-1.0/libusb.h>
#include "dcsc_setups/PendulumWrite.h"
#include "dcsc_setups/PendulumRead.h"
#include "dcsc_setups/usb_utils.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <string.h>
#include <math.h>
#include <cmath>
#include <signal.h>
#include <iostream>


FUGIPendulum pendulum(0);
dcsc_setups::PendulumSensors obs;
float zero_voltage_beam;
float zero_voltage_pendulum;
float max_voltage_beam = 0.0;
float max_voltage_pendulum = 0.0;
float shift_beam = 0.0;
float shift_pendulum = 0.0;
float velocity_beam = 0.0;
float velocity_pendulum = 0.0;
float velocity_beam_filtered = 0.0;
float velocity_pendulum_filtered = 0.0;
float velocity_beam_old = 0.0;
float velocity_pendulum_old = 0.0;
float angle_beam_old = 0.0;
float angle_pendulum_old = 0.0;
float angle_beam = 0.0;
float angle_pendulum = 0.0;
float rate = 300.0;
float cutoff_frequency = 200.0;


float lowPassFilter(float velocity_old, float velocity, float dt, float RC)
{
  float alpha = dt / (RC + dt);
  return alpha * velocity + (1 - alpha) * velocity_old;
}


float getAngle(float voltage, float zero_voltage, float max_voltage, float angle_old, float* shift)
{
  float angle = (voltage - zero_voltage) / (max_voltage) * 2 * M_PI + *shift;
  if (angle - angle_old > M_PI) {
    angle -= 2 * M_PI;
    *shift -= 2 * M_PI;
  }
  if (angle - angle_old < -M_PI) {
    angle += 2 * M_PI;
    *shift += 2 * M_PI;
  }
  return angle;
}


float calculateVelocity(float angle, float angle_old, float time)
{
  float velocity = (angle - angle_old) / time;
  return velocity;
}


void calculateVelocities(float angle_beam, float angle_pendulum, float time)
{
  angle_beam = lowPassFilter(angle_beam_old, angle_beam, time, 1 / (2 * M_PI * cutoff_frequency));
  angle_pendulum = lowPassFilter(angle_pendulum_old, angle_pendulum, time, 1 / (2 * M_PI * cutoff_frequency));
  velocity_beam = calculateVelocity(angle_beam, angle_beam_old, time);
  velocity_pendulum = calculateVelocity(angle_pendulum, angle_pendulum_old, time);
  if (velocity_pendulum > 50.0) {
    velocity_pendulum = velocity_pendulum_old;
  }
  if (velocity_pendulum < -50.0) {
    velocity_pendulum = velocity_pendulum_old;
  }
  angle_beam_old = angle_beam;
  angle_pendulum_old = angle_pendulum;
  velocity_beam_filtered = velocity_beam;
  velocity_pendulum_filtered = velocity_pendulum;
  velocity_beam_old = velocity_beam;
  velocity_pendulum_old = velocity_pendulum;

  // velocity_beam_filtered = lowPassFilter(velocity_beam_old, velocity_beam, time, 1 / (2 * M_PI * cutoff_frequency));
  // velocity_pendulum_filtered = lowPassFilter(velocity_pendulum_old, velocity_pendulum, time, 1 / (2 * M_PI * cutoff_frequency));
}


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
  res.sensors = obs;
  return true;
}


void __read()
{
  int status;
  if ((status = pendulum.read()) != EOK) {
    ROS_ERROR("[pendulum] Reading failed with status [%s], I/O error [%s]", strerror(status), strerror(pendulum.read_error()));
    return;
  }
  obs.header.stamp = ros::Time::now();
  obs.relative_time = pendulum.sensors.relative_time;
  obs.current = pendulum.sensors.current;  
  obs.position0 = pendulum.sensors.position0;
  obs.position1 = pendulum.sensors.position1;
  obs.voltage_beam = pendulum.sensors.voltage_beam;
  obs.voltage_pendulum = pendulum.sensors.voltage_pendulum;
  obs.digital_inputs = pendulum.sensors.digital_inputs;
  obs.current = pendulum.sensors.current;
  angle_beam = getAngle(obs.voltage_beam, zero_voltage_beam, max_voltage_beam, angle_beam_old, &shift_beam);
  angle_pendulum = getAngle(obs.voltage_pendulum, zero_voltage_pendulum, max_voltage_pendulum, angle_pendulum_old, &shift_pendulum);
  calculateVelocities(angle_beam, angle_pendulum, obs.relative_time);
  obs.angle_beam = angle_beam;
  obs.angle_pendulum = angle_pendulum;
  obs.velocity_beam = velocity_beam_filtered;
  obs.velocity_pendulum = velocity_pendulum_filtered;
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
  n.getParam("rate", rate);
  n.getParam("cutoff", cutoff_frequency);

  ros::Publisher obs_pub = n.advertise<dcsc_setups::PendulumSensors>("obs", 1);
  
  ros::Rate r(rate);

  // Override the default ros sigint handler.
  signal(SIGINT, mySigintHandler);

  bool initialized = false;
  unsigned int count = 0;
  int status;


  while (!initialized)
  {
    ++ count;
    if (count > 5)
    {
      ROS_ERROR("[pendulum] Maximum number of attempts to initialize Pendulum reached.");
      return 0;
    }
    initialized = init();
    r.sleep();
  }

  // Calibration
  string strOut("[pendulum] Please move the pendulum downwards and press enter.");
  cout << strOut << endl;
  std::string input;
  std::getline(std::cin, input);
  do {
     std::getline(std::cin, input);
  } while (input.length() != 0);

  // Read voltages
  float voltage_beam;
  float voltage_pendulum;

  if ((status = pendulum.read()) != EOK) {
    ROS_ERROR("[pendulum] Reading failed with status [%s], I/O error [%s]", strerror(status), strerror(pendulum.read_error()));
  }
  zero_voltage_beam = pendulum.sensors.voltage_beam;
  zero_voltage_pendulum = pendulum.sensors.voltage_pendulum;

  strOut = "[pendulum] Please rotate the inner and outer pendulum 360 degrees.";
  ros::Time begin = ros::Time::now();
  cout << strOut << endl;
  bool done = false;

  while (!done)
  {
    if ((ros::Time::now() - begin).toSec() > 10.0)
    {
      done = true;
    }
    if ((status = pendulum.read()) != EOK) {
      ROS_ERROR("[pendulum] Reading failed with status [%s], I/O error [%s]", strerror(status), strerror(pendulum.read_error()));
    }
    voltage_beam = pendulum.sensors.voltage_beam;
    voltage_pendulum = pendulum.sensors.voltage_pendulum;
    if (voltage_beam > max_voltage_beam)
    {
      max_voltage_beam = voltage_beam;
    }
    if (voltage_pendulum > max_voltage_pendulum)
    {
      max_voltage_pendulum = voltage_pendulum;
    }
    r.sleep();
  }

  strOut = "[pendulum] Calibration finished.\n Beam: zero = " + to_string(zero_voltage_beam) + ", max = " + to_string(max_voltage_beam) + "\n  Pendulum: zero = " + to_string(zero_voltage_pendulum) + ", max = " + to_string(max_voltage_pendulum) + "\n";
  cout << strOut << endl;

  __read();

  ros::ServiceServer write_service = n.advertiseService("write", write);
  ros::ServiceServer read_service = n.advertiseService("read", read);

  while (ros::ok())
  {
    __read();
    obs_pub.publish(obs);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
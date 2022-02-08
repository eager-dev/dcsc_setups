# DCSC Setups ROS package

This ROS package allows users to read from and write to the FPGA based DCSC setups.
Currently, only the MOPS setup is supported.

## Installation Instructions

1. Install the appropriate driver that is available in the [drivers folder](#drivers).

2. Clone this repository into a catkin workspace:
```bash
cd [path to catkin workspace]/src
git clone git@github.com:eager-dev/dcsc_setups.git
```

3. Build and source the workspace:
```bash
cd [path to catkin workspace]
catkin_make
source devel/setup.bash
```

## MOPS

### Usage

After plugging in the USB and putting on the device, the ROS interface can be activated by running:
```bash
roslaunch dcsc_setups mops.launch
```

### Example node

The following example shows how to perform PD control.
The example merely serves as a usage example and should not be used.
The gains are not tuned and performance is not tested.

```Python
#!/usr/bin/env python3

import rospy
from dcsc_setups.srv import MopsWrite, MopsWriteRequest, MopsRead, MopsReadRequest


class MopsExampleNode(object):
    def __init__(self):
        rospy.init_node('mops_node')

        self.mops_data = None
        self.rate = rospy.Rate(30)

        # Initialize request
        self.request = MopsWriteRequest()
        self.request.actuators.digital_outputs = 1
        self.request.actuators.voltage0 = 0.0  # Input voltage
        self.request.actuators.voltage1 = 0.0
        self.request.actuators.timeout = 0.5  # The timeout of the input (after 0.5 seconds the input will be set to 0)

        # Subscriber and service
        self.read_service = rospy.ServiceProxy('/mops/read', MopsRead)
        self.write_service = rospy.ServiceProxy('/mops/write', MopsWrite)

    def run(self):
        while not rospy.is_shutdown():
            # Get joint angle and velocity
            response = self.read_service(MopsReadRequest())
            joint_angle = response.sensors.position0
            joint_velocity = response.sensors.speed

            # PD control, be careful these values are made up!
            u = 1.0 - joint_angle - 0.05 * joint_velocity

            # Send command to MOPS
            self.request.actuators.voltage0 = u
            self.write_service(self.request)

            self.rate.sleep()


if __name__ == '__main__':
    mops_control_node = MopsExampleNode()
    mops_control_node.run()

```

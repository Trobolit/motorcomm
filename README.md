# motorcomm node
Motor communicator for [PermoCar](https://github.com/Trobolit/PermoCar) project.
These node subscribe to the power to give the engines to forward the information to a Arduino threw a serial communication link.
## Subscribe
* Topic motor_power, geometry_msgs::Twist (message form engien_mgmt)

## Publish
* Serial communication to ttyACM1 containing motor power

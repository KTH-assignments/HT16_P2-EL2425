# F1/10: control under high slip code repository

Here you will find the code used in the EL2425 - Automatic Control, Project Course, KTH, HT16 P2

---

## Running package `teleop`

```
ssh to Jetson (ssh ubuntu@vehicle-ip)
$ roscore
$ rosrun teleop keyboard.py
$ rosrun teleop key_receiver.py
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

---

## Running package `circular_pid`

Locally

```
$ export ROS_IP=192.168.3.110
$ roslaunch circular_pid starter.launch
$ rosrun circular_pid kill_switch.py

'Delete' to stop, 'Home' for normal operation

```
On Jetson

```
$ export ROS_MASTER_URI=http://192.168.3.110:11311
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

---

## Running package `centerline_pid`

Locally

```
$ export ROS_IP=192.168.3.110
$ roslaunch centerline_pid starter.launch
$ rosrun centerline_pid kill_switch.py

'Delete' to stop, 'Home' for normal operation

```
On Jetson

```
$ export ROS_MASTER_URI=http://192.168.3.110:11311
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```


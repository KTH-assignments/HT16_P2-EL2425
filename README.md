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

## Running package `circular_mpc`

Locally

```
$ export ROS_IP=192.168.3.110 (your IP over the picostation1 network)
$ roslaunch circular_mpc launcher.launch

Open another tab in your terminal
$ export ROS_IP=192.168.3.110 (your IP over the picostation1 network)
$ rosrun circular_pid kill_switch.py

(Press 'Delete' to stop, 'Home' for normal operation)

```
On Jetson

```
$ export ROS_MASTER_URI=http://192.168.3.110:11311 (your IP over the picostation1 network)
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

---

## Running package `circular_pid`

Locally

```
$ export ROS_IP=192.168.3.110 (your IP over the picostation1 network)
$ roslaunch circular_pid launcher.launch

Open another tab in your terminal
$ export ROS_IP=192.168.3.110 (your IP over the picostation1 network)
$ rosrun circular_pid kill_switch.py

(Press 'Delete' to stop, 'Home' for normal operation)

```
On Jetson

```
$ export ROS_MASTER_URI=http://192.168.3.110:11311 (your IP over the picostation1 network)
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

---

## Running package `centerline_mpc`

On Jetson

```
$ roslaunch centerline_mpc launcher.launch

$ rosrun centerline_pid kill_switch.py
(Press 'Delete' to stop, 'Home' for normal operation)

$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

---

## Running package `centerline_pid`

On Jetson

```
$ roslaunch centerline_pid launcher.launch

$ rosrun centerline_pid kill_switch.py
(Press 'Delete' to stop, 'Home' for normal operation)

$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

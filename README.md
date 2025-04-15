# Rover's Communication Setup Guideline

This `README.md` file goes through the steps needed to setup communication between Jetson nano i.e. rover and a laptop i.e. base station.

We use ROCKER M2 router for communication in both rover and the ground station.

.

.

The ash colored long ethernet cable(aka POE- Power Over Ethernet) is always connected to the ROCKET M2 router and the white comparatively small ethernet cable is connected to laptop(in case of base station sector antenna) or jetson nano(in case of rover).

Whenever testing the rover using communication setup, make sure that both the ROCKET M2 is at least connected to dipole antenna or better if their respective sector/omnidirectional antenna. But at least one antenna should stay connnected to the router all the time during testing.

After connecting the wires properly to the base station antenna and the laptop, we need to configure our network settings(you need to repeat the same steps in nano after connecting wires properly to the jetson and ROCKET M2 router):

![Terminal Preview](rover-communication_setup/assets/ethernet_config2.png)

By default, IPv4 Method will be set to 'Automatic(DHCP)', we need to select 'manual'

And  we need to type the Addresses.
In the **Adrress** type `192.168.1.100` in case of base station. The same thing when dealing with jetson is `192.168.1.2`

Netmask is `255.255.255.0` same for both laptop and jetson.

After this, close and reopen the wired settings, now you should see something like this:

![Terminal Preview](rover-communication_setup/assets/ethernet_config1.png)

---
# 🛠️ Procedure

Currently we have our laptop connected to base station's ROCKET M2

To verify this, open terminal and run:

```bash
hostname -I
```
In terminal you should see something like this:
```bash
192.168.1.100
```

The jetson(in this case, nano) is also connected to a ROCKET M2
Similary, to verify/access its ip address, run this in the nano:
```bash
hostname -I
```
You should see something like this:
```bash
192.168.1.2
```
---

## 📱💻 Ping

Now we want to verify that both our jetson nano and laptop(base station) is connected to the same network.
For this, we'll try to ping the jetson using our laptop.
Run:
```bash
ping 192.168.1.2
```

Upon successful ping, you should see something like this:

---
![Terminal Preview](rover-communication_setup/assets/ping_jetson.png)

⚠️ Remember, our base station(i.e. laptop) ip address is 192.168.1.100

If you see the above output, you can safely rest assure that your jetson and base station is connected in the same network.

## 📱💻 SSH

Now that both our jetson nano and the base station(laptop) are in the same network,
we can access the jetson(that is situated in the rover) by a process called 'ssh'

WRITE THIS IN YOUR TERMINAL:

```bash
ssh interplanetar@192.168.1.2
```
This will prompt you to input password of the jetson nano
After typing the correct password, your terminal should look something like this:

---
![Terminal Preview](rover-communication_setup/assets/ssh_jetson.png)

---


⚠️ Take a good look, the username and other credentials have changed to interplanetar after successful ssh
It shows that our ssh was successful and now we have complete access to the jetson nano
We can try this command:
```bash
ls
```
It will show the workspaces and file that are inside the jetson

## Docker inside Jetson

We have docker installed in our jetson nano
We can see the container list using this following command:
```bash
docker ps -a
```
---
You should see a container named 'humble-nano'. This is the container that we will be using.

Now start the container:

```bash
docker start -i humble-nano
```
---
![Terminal Preview](rover-communication_setup/assets/docker_ros_humble.png)

---
On successful running, you should see the username getting changed to roverNano which tells us that we indeed are now inside the ros2 humble container, and now we can do all sorts of stuff that we want using ROS2

---
## Running a node in Laptop i.e. Base station

Now create a new window in terminal.
And from there run any ros2 node you want(Here's a demo version)
Here we ran the GUI node of the rover:
```bash
cd ros2_nafi_ws
source install/setup.bash
ros2 run combined_gui gui_node
```
![Terminal Preview](rover-communication_setup/assets/laptop_ros2_run.png)


Now in another window, run this:
```bash
ros2 topic list
```
![Terminal Preview](rover-communication_setup/assets/ros2_topic_laptop.png)

## Topic list in ros-humble inside Docker

Now in the terminal where ssh i.e. jetson nano i.e. 'humble-nano' container is running, write the same command:
```bash
ros2 topic list
```
See, they are the same topic that are in the laptop i.e. base station. Which tells us that our jetson can now access those same topic.

![Terminal Preview](rover-communication_setup/assets/docker_ros_humble.png)


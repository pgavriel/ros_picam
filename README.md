# ROS RASPBERRY PI CAMERAS (ROS_PICAM)    
A package that enables the usage of Raspberry Pi's using camera module V2 as ROS connected network cameras. Raspberry Pi setup detailed below. The Picams can be set up to launch the client automatically and wait for a master roscore to come online. The client continually checks that the master is running, and if not, it will shut itself down and continually try to restart itself. This allows for a virtually hands off approach, all you have to do is turn the Raspberry Pi on and wait for everything to start.(Automatic startup not yet detailed) The client uses ROS services to tell the camera to take still images or record video. Files are saved locally on the Pi and can be transferred elsewhere via FTP or some other means.   

### Raspberry Pi Setup  
#### Step 1: Install Ubiquity Robotics Raspberry Pi ROS Image    
https://downloads.ubiquityrobotics.com/pi.html    
Any Raspberry Pi with a working ROS installation should be able to work, but this image comes with everything already installed which makes the process a lot easier.  

#### Step 2: Network / Hostname setup    
Log into the Pi and connect to your desired network. If you intend on using multiple cameras, you should also set a unique hostname for each Pi.  
[Ubiquity Network Setup Tutorial](https://learn.ubiquityrobotics.com/connect_network)   

#### Step 3: Setup ROS_MASTER_URI   
I tend to add the ROS_MASTER_URI export line to the bottom of my ~/.bashrc so you don't need to type it repeatedly.   
[ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup)   
[ROS Tutorial: Multiple Machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)   

#### Step 4: Clone this repository to the Pi's ROS workspace  
**NOTE:** Ubiquity Robotics also has a [raspicam_node repository](https://github.com/UbiquityRobotics/raspicam_node) that will stream camera data over ROS topics. Their repository might also be useful to clone also, however this repository is not integrated with it in any way *yet*.   
Make sure to build the workspace afterwards.   

#### Step 5: Clone this repository to the master machines workspace  
This is so the services can be called from the master machine. Otherwise, it will see the services but will throw an error when calling them saying it doesn't know how the service is defined.

#### Step 6: [Optional] Configure the Pi to login automatically on startup    
From [this forum post](https://forum.ubiquityrobotics.com/t/a-hack-to-autologin/236/4), the solution from jonovos worked well for me.  
Create the following file:     
```
sudo touch /etc/lightdm/lightdm.conf.d/10-autologin.conf
```      

Edit the file:    
```
sudo vim /etc/lightdm/lightdm.conf.d/10-autologin.conf
```   

Add the following contents:     
```   
[Seat:*]
autologin-guest = false
autologin-user = ubuntu
autologin-user-timeout = 0

[SeatDefaults]
allow-guest =  false
```   
Reboot the raspberry pi and it should automatically log you back in.   

#### Step 7: [Optional] Create a startup service for picam client
**Background:**     
Adapted from instructions [here](https://risc.readthedocs.io/2-auto-service-start-afer-boot.html). However, following these instructions alone, the service was not launching the picam node. I found the [robot_upstart](http://wiki.ros.org/robot_upstart) package, and tried to use that to create the startup service, and after lots of troubleshooting, the service it created seemed to get a step further, but the node was still failing due to [an error like this](https://stackoverflow.com/questions/42583835/failed-to-open-vchiq-instance), and I couldn't find any good solutions that worked, so I started poking around to see how robot_upstart was doing things. The final implementation I ended up with is a Frankenstein between the first set of instructions and pieces of script generated by robot_upstart, but it seems to be working well and is straightforward to setup.  
**Instructions:**  
I recommend reading through *create_startup_service.sh*, *picam_autostart.service*, and *startup_launch.sh* to try to understand what's going on (especially *startup_launch.sh* because some values may need to be changed such as ROS_MASTER_URI). I also recommend briefly looking into [systemctl](https://www.commandlinux.com/man-page/man1/systemctl.1.html) if you aren't already somewhat familiar.   
**ON THE RASPBERRY PI**, calling *create_startup_service.sh* and rebooting the machine should be all you have to do to create the service and set it to automatically start itself.   
```   
cd ~/catkin_ws/src/ros_picam/scripts  
./create_startup_service.sh   
sudo reboot  
```    
You can check the status of the service like so:   
```   
systemctl status picam_autostart
```   
If you're seeing errors, you likely need to make some changes to *startup_launch.sh*, and you can apply any changes you make (or restart the service for any other reason) like so:  
```   
sudo systemctl restart picam_autostart  
```   
NOTE: For currently unknown reasons, text output from the node only makes it into systemctl status when the node is shutdown. Ideally there will be a way around this, or some other way of monitoring the nodes output in realtime for troubleshooting.

### Using ros_picam
The main script being used is **picam_client.py**, the other python scripts in the src folder are just for testing purposes and will likely be removed at some point.
#### On the Pi   
**client.launch** is used. The name for the camera and the directory to save captures can be specified. The *--wait* flag can be used to ensure it waits for roscore to be available before launching:   
```
roslaunch ros_picam client.launch --wait
```   
This node will continually try to restart itself if the connection to master is lost.  
#### On the master machine   
The three currently implemented services are **grab_still**, **start_recording**, and **stop_recording**. They can be called via rosservice like:
```
rosservice call [node_name]/grab_still 3
```   
However, the launch files provided will also call their respective services. The benefit of calling the services via launch files is that they can easily be edited to call the services from multiple Picams simultaneously if using a multi-camera setup.   

Additional services can be defined and implemented as needed.  

TODO: Recorded videos may all need to be converted using something like MP4Box (part of gpac). [This was the most relevant post I found](https://www.raspberrypi.org/forums/viewtopic.php?t=245875) regarding choppy/glitched video playback, and it seems like the data itself is fine, but it lacks some information for proper playback for some reason. Will likely address this when it becomes clearer how stills and videos are going to be handled over the network.

### Additional Notes   
The scripts temp_check.sh and temp_monitor.sh can be copied into the **/usr/bin** folder so they may be called with the commands **tempcheck** and **tempmonitor** respectively to keep an awareness of the pi's core temperature under different conditions. Future plans include the ability to publish this information over ROS topics.  

If you have a roscore running on your master machine and the raspberry pi can see the topics listed, **make sure the raspberry pi is actually receiving data on these topics**. If not, the IP address and hostname of the master machine need to be added to the Pi's **/etc/hosts** file. If the IP address of the master machine ever changes, these issues will pop up again. Resolve them by setting the correct IP address in the hosts file.   

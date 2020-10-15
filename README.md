# ROS RASPBERRY PI CAMERAS (ROS_PICAM)    
A package that enables the usage of Raspberry Pi's using camera module V2 as ROS connected network cameras. Raspberry Pi setup detailed below. The Picams can be set up to launch the client automatically and wait for a master roscore to come online. The client continually checks that the master is running, and if not, it will shut itself down and continually try to restart itself. This allows for a virtually hands off approach, all you have to do is turn the Raspberry Pi on and wait for everything to start.(Automatic startup not yet detailed) The client uses ROS services to tell the camera to take still images or record video. Files are saved locally on the Pi and can be transferred elsewhere via FTP or some other means.   

### Raspberry Pi Setup  
##### Step 1: Install Ubiquity Robotics Raspberry Pi ROS Image    
https://downloads.ubiquityrobotics.com/pi.html     

##### Step 2: Network / Hostname setup    
Log into the Pi and connect to your desired network. If you intend on using multiple cameras, you should also set a unique hostname for each Pi.   

##### Step 3: Configure the Pi to login automatically on startup    
From [this forum post](https://forum.ubiquityrobotics.com/t/a-hack-to-autologin/236/4), the solution from jonovos worked well for me.  
Create the following file:     
> sudo touch /etc/lightdm/lightdm.conf.d/10-autologin.conf      

Edit the file:    
>sudo vim /etc/lightdm/lightdm.conf.d/10-autologin.conf    

Add the following contents:     
```   
[Seat:*]
autologin-guest = false
autologin-user = ubuntu
autologin-user-timeout = 0

[SeatDefaults]
allow-guest =  false
```   

##### Step 4: Setup ROS_MASTER_URI   
[ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup)   
[ROS Tutorial: Multiple Machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)   

##### Step 5: Clone this repository to the Pi's ROS workspace  
**NOTE:** Ubiquity Robotics also has a [raspicam_node repository](https://github.com/UbiquityRobotics/raspicam_node) that will stream camera data over ROS topics. Their repository might also be useful to clone also, however this repository is not integrated with it in any way *yet*.   
Make sure to build the workspace afterwards.   

##### Step 6: Clone this repository to the master machines workspace  
This is so the services can be called from the master machine. Otherwise, it will see the services but will throw an error when calling them saying it doesn't know how the service is defined.



### Additional Notes   
The scripts temp_check.sh and temp_monitor.sh can be copied into the **/usr/bin** folder so they may be called with the commands **tempcheck** and **tempmonitor** respectively to keep an awareness of the pi's core temperature under different conditions. Future plans include the ability to publish this information over ROS topics.  

If you have a roscore running on your master machine and the raspberry pi can see the topics listed, **make sure the raspberry pi is actually receiving data on these topics**. If not, the IP address and hostname of the master machine need to be added to the Pi's **/etc/hosts** file. If the IP address of the master machine ever changes, these issues will pop up again. Resolve them by setting the correct IP address in the hosts file.   

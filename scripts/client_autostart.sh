#!/bin/bash
touch picam_startup.sh
echo "#!/bin/bash" >> picam_startup.sh
echo "roslaunch ros_picam client.launch --wait" >> picam_startup.sh
chmod +x picam_startup.sh
sudo mv picam_startup.sh /etc/init.d/picam_startup.sh

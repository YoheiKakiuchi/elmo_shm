### install library
 - smb://andromeda/share1/STARO/downloads/liblpsensor-1.3.5-Linux.deb
 - sudo dpkg -i liblpsensor-1.3.5-Linux.deb

### install udev rules
 - sudo cp 81-lpms-imu.rules /etc/udev/rules.d/
 - sudo service udev restart

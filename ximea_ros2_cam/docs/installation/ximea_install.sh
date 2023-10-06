#!/bin/bash

# NOTE: you may need a reboot for some of these changes to take effect

# Download and extract the most recent software package:
cd ~; mkdir tmp; cd tmp
wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz

tar xzf XIMEA_Linux_SP.tgz
cd package

# Install the package, can change interface type depending on your camera
./install -cam_usb30

# Add user to the pugdev group:
sudo gpasswd -a $USER plugdev

# Set the USB FS memory allocation to infinite for sufficient buffering size
# for high bandwith USB3.0 streams:

echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb
# You can put this line to your bashrc file to apply to every new shell

# set realtime priority
echo "*               -       rtprio          0" >> /etc/security/limits.conf
echo "@realtime       -       rtprio          81" >> /etc/security/limits.conf
echo "*               -       nice            0" >> /etc/security/limits.conf
echo "@realtime       -       nice            -16" >> /etc/security/limits.conf

# add the user to realtime
sudo groupadd realtime #if it doesn't exist yet
sudo gpasswd -a $USER realtime
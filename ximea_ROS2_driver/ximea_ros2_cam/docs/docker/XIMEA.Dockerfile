# NOTE: this script only downloads and sets-up the software
#       package. You need to edit the "user" container to add
#       the admin user to the plugdev and realtime groups since this
#       requires the user's username, GID and UID.

#       SEE: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docker/Dockerfile.user 

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

# Download and extract the ximea software package
RUN wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz

RUN cd package && /install -cam_usb30

# Set the USB FS memory allocation to infinite for sufficient buffering size \
# for high bandwith USB3.0 streams:

RUN echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb
# You can put this line to your bashrc file to apply to every new shell

# set realtime priority
RUN echo "*               -       rtprio          0" >> /etc/security/limits.conf
RUN echo "@realtime       -       rtprio          81" >> /etc/security/limits.conf
RUN echo "*               -       nice            0" >> /etc/security/limits.conf
RUN echo "@realtime       -       nice            -16" >> /etc/security/limits.conf

# USER setup bash equivalents
# sudo gpasswd -a $USER plugdev
# sudo groupadd realtime #if it doesn't exist yet
# sudo gpasswd -a $USER realtime
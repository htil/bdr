FROM ros:kinetic

WORKDIR /bdr
COPY ./bdr_ws /bdr/

# Install ROS dependencies and build
RUN apt-get update && \
	apt-get install screen vim git python-pip -y && \
	/usr/bin/python -m pip install --upgrade pip && \
	apt-get remove python-pip -y && \
	/usr/bin/python -m pip install pyrebase && \
	rosdep install --from-paths ./src --ignore-src -y && \
	. /opt/ros/kinetic/setup.sh && \
	(set +e; /opt/ros/kinetic/bin/catkin_make; /opt/ros/kinetic/bin/catkin_make; set -e) && \
	echo "alias screen=\"screen bash\"" >> /root/.bashrc && \
	echo "alias land=\"rostopic pub /bebop/land std_msgs/Empty\"" >> /root/.bashrc && \
	echo "alias calibrate_bebop_camera=\"rostopic pub /bebop/camera_control geometry_msgs/Twist \\\"{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: -90, z: 0}}\\\"\"" >> /root/.bashrc && \
	echo ". /bdr/devel/setup.bash" >> /root/.bashrc

# Install web dependencies
WORKDIR /
RUN sh -c "$(curl -sL https://raw.githubusercontent.com/creationix/nvm/v0.33.8/install.sh)" && \
	. ~/.profile && \
	nvm install 11.0.0 && \
	git clone https://github.com/htil/bdr-muse && \
	echo "export NVM_DIR=\"$HOME/.nvm\"\n[ -s \"$NVM_DIR/nvm.sh\" ] && \\. \"$NVM_DIR/nvm.sh\"  # This loads nvm" >> /root/.bashrc && \
	cd /bdr-muse && \
	npm install

# Allow for the musejs control center to connect
EXPOSE 8888

# #
# FROM ros:kinetic

# # Grab necessary build files
# WORKDIR /
# COPY --from=build /bdr ./bdr
# COPY --from=build /bdr-muse ./bdr-muse
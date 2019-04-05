FROM ros:kinetic

WORKDIR /bdr
COPY ./bdr_ws /bdr/

# Install ROS dependencies and build
RUN apt-get update && \
	apt-get install screen vim git -y && \
	rosdep install --from-paths ./src --ignore-src -y
	# (set +e; catkin_make -j1; catkin_make; set -e)

# Install web dependencies
WORKDIR /
RUN sh -c "$(curl -sL https://raw.githubusercontent.com/creationix/nvm/v0.33.8/install.sh)" && \
	. ~/.profile && \
	nvm install 11.0.0 && \
	git clone https://github.com/htil/bdr-muse

# Allow for the musejs control center to connect
EXPOSE 8888
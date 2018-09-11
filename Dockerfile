FROM bitbots/bitbots-common:kinetic

WORKDIR /kinetic
COPY mas-navigation.rosinstall /kinetic

ADD . /kinetic/src/mas_navigation

RUN . /opt/ros/mas_stable/setup.sh && \
    apt-get update -qq && \
    rosdep update -q && \
    rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y && \
    rm -rf /var/lib/apt/lists/* && \
    catkin config --init && \
    catkin config --extend /opt/ros/mas_stable && \
    catkin config --install --install-space /opt/ros/mas_stable && \
    catkin build && \
    rm -rf /kinetic/

WORKDIR /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

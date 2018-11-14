FROM gazebo:gzserver9

RUN mkdir /root/gazebo_data

COPY ./world/interview_city.world /root/gazebo_data/interview_city.world
COPY ./world/libworld_sim.so /root/gazebo_data/libworld_sim.so
COPY ./world/libworld_state_msgs.so /root/gazebo_data/libworld_state_msgs.so

# We need to set the plugin path AND be in the directory where the plugins are,
# otherwise Gazebo can not find the second pluging
ENV GAZEBO_PLUGIN_PATH /root/gazebo_data
WORKDIR /root/gazebo_data

CMD [ "gzserver", "/root/gazebo_data/interview_city.world" ]

FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /app

COPY ./src keep-alive-test/

RUN cd keep-alive-test && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
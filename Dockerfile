FROM osrf/ros:humble-desktop AS builder
SHELL ["/bin/bash", "-c"]
WORKDIR /app

COPY ./src ./src

RUN source /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

FROM osrf/ros:humble-desktop AS runtime
SHELL ["/bin/bash", "-c"]
WORKDIR /app

COPY --from=builder /app/install /app/install

COPY src/scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
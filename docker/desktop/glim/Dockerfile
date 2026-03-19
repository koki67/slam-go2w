ARG GLIM_BASE_IMAGE=koide3/glim_ros2:jazzy_cuda13.1
FROM ${GLIM_BASE_IMAGE}

SHELL ["/bin/bash", "-lc"]

COPY docker/desktop/glim/bin/prepare_glim_config.sh /opt/go2w/bin/prepare_glim_config.sh
COPY docker/desktop/glim/bin/run_glim_rosbag.sh /opt/go2w/bin/run_glim_rosbag.sh

RUN chmod +x /opt/go2w/bin/prepare_glim_config.sh /opt/go2w/bin/run_glim_rosbag.sh \
    && mkdir -p /opt/glim_ros2 \
    && cp -a /root/ros2_ws/install /opt/glim_ros2/install \
    && chmod -R a+rX /opt/glim_ros2 \
    && mkdir -p /work/input /work/config_override /work/runtime_config /work/output/logs /work/output/results

ENTRYPOINT ["/bin/bash"]

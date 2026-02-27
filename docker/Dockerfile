FROM ros:jazzy-ros-base

# ── System deps ───────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros2-control \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-robot-localization \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ── Python deps (VLM + plotting) ─────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    transformers>=4.40 \
    torch --index-url https://download.pytorch.org/whl/cpu \
    opencv-python-headless \
    matplotlib \
    numpy \
    scipy

# ── Workspace ─────────────────────────────────────────────────────────────────
WORKDIR /ws
COPY src/ src/
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

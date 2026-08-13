FROM osrf/ros:noetic-desktop-full

# 1. Instalar dependencias de ROS, Python y utilidades del sistema
RUN apt-get update && apt-get install -y \
    git \
    nano \
    net-tools \
    iputils-ping \
    python3-catkin-tools \
    python3-pip \
    python3-tk \
    python3-pykdl \
    ros-noetic-kdl-parser-py \
    ros-noetic-serial \
    && rm -rf /var/lib/apt/lists/*

# 2. Instalar paquetes de Python necesarios
RUN pip3 install --no-cache-dir \
    pybullet \
    -U "numpy<2.0" \
    urdfdom-py \
    pillow \
    scipy

# 3. Variable global de Python
ENV PYTHONPATH="${PYTHONPATH}:/workspace"

# 4. Configurar el ~/.bashrc interno del contenedor
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' ~/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo 'if [ -f /workspace/catkin_ws/devel/setup.bash ]; then source /workspace/catkin_ws/devel/setup.bash; fi' >> ~/.bashrc && \
    echo 'if [ -f /workspace/setup.py ] || [ -f /workspace/pyproject.toml ]; then pip install --no-build-isolation -e /workspace >/dev/null 2>&1; fi' >> ~/.bashrc

WORKDIR /workspace
CMD ["bash"]
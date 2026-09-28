#!/bin/bash
set -e

# Cargar entorno de ROS Noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
fi

# Cargar entorno del catkin_ws si existe
if [ -f /workspace/catkin_ws/devel/setup.bash ]; then
    source /workspace/catkin_ws/devel/setup.bash
fi

# 1. Iniciar pantalla virtual de Xvfb (1920x1080)
Xvfb :0 -screen 0 1920x1080x24 &
export DISPLAY=:0
sleep 1

# 2. Iniciar gestor de ventanas
fluxbox &

# 3. Iniciar servidor VNC en puerto 5900
x11vnc -display :0 -forever -shared -rfbport 5900 -nopw &

# 4. Iniciar servidor web noVNC en puerto 6080
websockify --web /usr/share/novnc 6080 localhost:5900 &

# 5. Ejecutar la orden por defecto (bash u otro comando ROS)
exec "$@"
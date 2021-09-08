#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/nidhi/ws_moveit/src/vb_simulation_pkgs/gazebo_ros_pkgs/gazebo_ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/nidhi/ws_moveit/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/nidhi/ws_moveit/install/lib/python2.7/dist-packages:/home/nidhi/ws_moveit/build/gazebo_ros/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/nidhi/ws_moveit/build/gazebo_ros" \
    "/usr/bin/python2" \
    "/home/nidhi/ws_moveit/src/vb_simulation_pkgs/gazebo_ros_pkgs/gazebo_ros/setup.py" \
     \
    build --build-base "/home/nidhi/ws_moveit/build/gazebo_ros" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/nidhi/ws_moveit/install" --install-scripts="/home/nidhi/ws_moveit/install/bin"
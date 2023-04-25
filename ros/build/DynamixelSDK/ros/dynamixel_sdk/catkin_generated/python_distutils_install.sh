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

echo_and_run cd "/home/turtle/ECN-2-1-Robot/ros/src/DynamixelSDK/ros/dynamixel_sdk"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/turtle/ECN-2-1-Robot/ros/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/turtle/ECN-2-1-Robot/ros/install/lib/python3/dist-packages:/home/turtle/ECN-2-1-Robot/ros/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/turtle/ECN-2-1-Robot/ros/build" \
    "/usr/bin/python3" \
    "/home/turtle/ECN-2-1-Robot/ros/src/DynamixelSDK/ros/dynamixel_sdk/setup.py" \
     \
    build --build-base "/home/turtle/ECN-2-1-Robot/ros/build/DynamixelSDK/ros/dynamixel_sdk" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/turtle/ECN-2-1-Robot/ros/install" --install-scripts="/home/turtle/ECN-2-1-Robot/ros/install/bin"

#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/biorobotics/Nico/daVinci/qt/src/rqt_example_cpp"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/biorobotics/Nico/daVinci/qt/install/lib/python2.7/dist-packages:/home/biorobotics/Nico/daVinci/qt/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/biorobotics/Nico/daVinci/qt/build" \
    "/usr/bin/python" \
    "/home/biorobotics/Nico/daVinci/qt/src/rqt_example_cpp/setup.py" \
    build --build-base "/home/biorobotics/Nico/daVinci/qt/build/rqt_example_cpp" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/biorobotics/Nico/daVinci/qt/install" --install-scripts="/home/biorobotics/Nico/daVinci/qt/install/bin"

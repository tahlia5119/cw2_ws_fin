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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/tahlia/cw2_redo/src/comp313p/comp313p_mapper"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/tahlia/cw2_redo/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/tahlia/cw2_redo/install/lib/python2.7/dist-packages:/home/tahlia/cw2_redo/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/tahlia/cw2_redo/build" \
    "/usr/bin/python" \
    "/home/tahlia/cw2_redo/src/comp313p/comp313p_mapper/setup.py" \
    build --build-base "/home/tahlia/cw2_redo/build/comp313p/comp313p_mapper" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/tahlia/cw2_redo/install" --install-scripts="/home/tahlia/cw2_redo/install/bin"

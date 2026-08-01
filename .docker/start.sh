xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "/home/jan/git/basalt:/workspace/basalt" -v "/home/jan/data/spot_calib_x5:/data" basalt

cd basalt
./thirdparty/vcpkg/bootstrap-vcpkg.sh -disableMetrics
cmake --preset relwithdebinfo
cmake --build --preset relwithdebinfo -j8
ctest --preset relwithdebinfo

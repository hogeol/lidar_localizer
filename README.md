# lidar_localizer

follow below link to download geographic library

https://github.com/geographiclib/geographiclib

> Open terminal

```
cd
mkdir libraries
cd libraries
git clone --recursive https://github.com/geographiclib/geographiclib.git
cd geographiclib
mkdir build && cd build
cmake ../
make -j$(nproc-4)
sudo make install
```

> In arguments ndt_max_thread

more thread is not always good results.

In my case, CPU i7-12700KF have 20-threads

but, Compare between thread 10 and 5, 5 is better in my environment

Plz find your best threads through experience


```
Cause it cannot support 'map-server' in humble, 

I don't know how to load HD map in rviz2

If you can load HD map in ros2, please commit that method

Thank you
```

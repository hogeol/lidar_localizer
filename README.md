# LiDAR_localizer

#### Follow below link to download geographic library

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

#### In arguments ndt_max_thread
```
more thread is not always good results.
In my case, CPU i7-12700KF have 20-threads
but, Compare between thread 10 and 5, 5 is better in my environment
Plz find your best threads through experience
```

#### Load HD Map
```
After execute the launch file,
open another terminal, and enter below command
$ ros2 run nav2_util lifecycle_bringup map_server
```

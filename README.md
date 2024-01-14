# lidar_localizer

## How to install third party

### geographic library

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

### Others

Ohter third party packages are included in this repository.

## Result

### Configuration launch file

> In arguments ndt_max_thread

more thread is not always good results

In my case, CPU i7-12700KF have 20-threads

but, Compare between thread 12 and above, it seems it doesn't have any difference in performance

I think it is almost same above 12-thread

In my case in ch.64 velodyne with car velocity is about 100km/h, performed well

Please find your best threads through experience

### Result with Ch. 32 velodyne LiDAR



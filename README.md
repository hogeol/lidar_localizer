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

## Execution

### Configuration launch file

> ndt_based_localizer.launch

In arguments **ndt_max_thread**.

Many thread is not always good results.

In my case, CPU i7-12700KF have 20-threads.

but, Compare between thread 12 and above, it seems it doesn't have any difference in performance.

I think it is almost same above 12-thread.

In my case in ch.64 velodyne with car velocity is about 100km/h, performed well.

Please find your best threads through experience.

***Recommend at least 4 thread are left for other process***

### GNSS to UTM conversion

Using geographic library for GNSS to UTM conversion.<br>
**Please check your UTM zone and implement at utm_zone parameter in launch file.** (south korea is 52)

<img src="https://github.com/hogeol/lidar_localizer/blob/master/media/GNSS2utm.gif" width=720px height=360px>
<br>

## Result

### Result compared with Ground truth

<img src="https://github.com/hogeol/lidar_localizer/blob/master/media/k_city_hd.png" width=240px height=606px> <img src="https://github.com/hogeol/lidar_localizer/blob/master/media/k_city_result.png" width=240px height=606px>
<br>
<**Green line** is localization result with LiDAR, **Red line** is ground truth>
<br>
<img src="https://github.com/hogeol/lidar_localizer/blob/master/media/cheongju_3d.png" width=404px height=240px> <img src="https://github.com/hogeol/lidar_localizer/blob/master/media/cheongju_result.png" width=404px height=240px>
<br>
<**Green line** is localization result with LiDAR, **Blue line** is ground truth>
<br>
<Using **dlo slam** to make point cloud map ([direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry))>

### Result without any filter
<img src="https://github.com/hogeol/lidar_localizer/blob/master/media/Localization_in_k_city.gif" width=720px height=360px>

<Without any filter, the localizer only performed accurately up to 40kph>

### Result with Exponential weight filter

<img src="https://github.com/hogeol/lidar_localizer/blob/master/media/cheongju_120kph_straight.gif" width=404px height=240px> <img src="https://github.com/hogeol/lidar_localizer/blob/master/media/cheongju_120kph_curve.gif" width=404px height=240px>

<With Exponential weight filter, the localizer can performed accurately up to 120kph>

## Future Work

### Filters
1. **LiDAR input date pre-correction** is needed if possible.   
   (X-axis based on robot frame is slightly late compared with input data. Currently, it is covered with **Exponential weight filter**.)
2. **Extended Kalman filter** is needed for more accurate localizer.

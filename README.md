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

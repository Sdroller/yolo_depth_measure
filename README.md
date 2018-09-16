# Yolo Depth Measurement

## Depth Measurement
This package determines the distance to a person. It subscribes the the bounding boxes output by Yolo and the depth image from the Zed camera and calculates the distance to the person from the centroid of the bounding box.

To make the readings more robust, 2 techniques are used:
- Distance is averaged from 10x10px around the centroid of the bounding box.
- The distance reading may be noisy. It is filtered using an exponential moving average.

There are 2 implementations of an exponential moving average filter in the code, any one may be used. They are both first order filters with similar performance:
- The simplest filter is called `moving average` or `ma` in the code. It has the formula:
```
new_filterOutput = (1-percentage_weightage)*old_filterOutput + (percentage_weightage)*nextElement
```
![](images/filter_ma_output.png)
- The other is called `Exponential Moving Average` or `ema` in code, calculated with a time constant.
```
new_filterOutput = old_filterOutput + (T/Tf)*(nextElement - old_filterOutput)
```
![](images/filter_ema_output.png)

### Steps to execute:
Launch the driver :
```
$ rosrun yolo_depth_measure yolo_depth_measure.py
```

## Object Tracking
Objects are detected by our deep learning algorithm and bounding boxes are generated for each detection. We need to track our target to follow. For this purpose, we use [SORT](https://arxiv.org/pdf/1602.00763.pdf) ([code](https://github.com/Shreeyak/sort)). To see more info, check out [tracking.md](./tracking.md).

### SORT Dependencies
##### Numba Install
As seen in the readme, `sort` has some dependencies to be installed:
```
$ pip install numba
```
There were problems, tried all of the following. Good tips in case there are problems with pip.
In case pip install gets stuck after downloading whl file at setup.py, try this command (may want to use `-v` or `sudo -H`):
```
$ pip install --upgrade setuptools
$ pip install --upgrade pip
```
For major issues, try downgrading pip:
```
$ sudo pip install --upgrade pip==9.0.3
```

##### Alternate: Install using apt-get
```
$ sudo apt install python-scipy
$ sudo apt install llvm-6.0
$ export LLVM_CONFIG=/usr/lib/llvm-6.0/bin/llvm-config
$ sudo apt install libedit-dev
```
This didn't help either. Looks like we'll have to build manually.

##### Soln: build manually
Check this thread: http://ai.sensilab.monash.edu/2018/08/31/Jetson-speech/
https://gist.github.com/jed-frey/ba40ff83523296bc8355f05befb28da9
http://www.athenian-robotics.org/jetsontx2/

So, download the sources from here: http://www.linuxfromscratch.org/blfs/view/svn/general/llvm.htmld
Preferable, add a 32gb pendrive to tx2, move sources there to build. Full build is around 29Gb, no enough space in inbuilt memory on tx2.
make a "build" directory inside the source directory and from the build directory, invoke command:
```
$ cmake .. -DCMAKE_BUILD_TYPE=Release -DLLVM_TARGETS_TO_BUILD="ARM;X86;AArch64"

#Start the build from the build directory
$ cmake -j4 --build .

#install it from the build directory:
$ cmake --build . --target install

$ cmake -DCMAKE_INSTALL_PREFIX=/tmp/llvm -P cmake_install.cmake
```

# Object Road Detection
This package provides a object road detector for AIFormula.
This package is based on [YOLOP](https://github.com/hustvl/YOLOP)

![image](https://github.com/user-attachments/assets/078b5c81-7e08-4ad7-9dc2-9ec09a537de5)

## YOLOP
```
cd ~/workspace/ros/src/aiformula/perception/yolop/yolop
```
### requirements
See requirements.txt for additional dependencies and version requirements.
```
pip3 install -r requirements.txt
```

## pytorch install

### requirement packages

```
    sudo apt-get -y install \
     autoconf bc build-essential g++-8 gcc-8 clang-8 lld-8 gettext-base gfortran-8 iputils-ping libbz2-dev libc++-dev libcgal-dev libffi-dev libfreetype6-dev libhdf5-dev libjpeg-dev liblzma-dev libncurses5-dev libncursesw5-dev libpng-dev libreadline-dev libssl-dev libsqlite3-dev libxml2-dev libxslt-dev locales moreutils openssl python-openssl rsync scons python3-pip libopenblas-dev
```

### pytorch install
* Specifying the Torch Version

    ```
    export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v502/pytorch/torch-1.13.0a0+d0d6b1f2.nv22.10-cp38-cp38-linux_aarch64.whl
    export "LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:$LD_LIBRARY_PATH"
    ```

* install

    ```
    python3 -m pip install --upgrade protobuf
    python3 -m pip install --no-cache $TORCH_INSTALL
    ```

### torchvision

install torchvision v0.13.0
https://github.com/dusty-nv/jetson-containers/blob/L4T-R35.1.0/Dockerfile.pytorch#L62-L84

```
git clone --branch release/0.15 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.13.0
python3 setup.py install --user
```
### Environment setup
Make sure you have pip and setuptools installed. Then:
```
cd ${aiformula}/perception/yolop
pip install -e .
```

Set up your environment by export the following command.

```sh
export PYTHONPATH=${aiformula}/perception/yolop:$PYTHONPATH
```

## Running Example:

* To start object road detector
```
ros2 launch object_road_detector object_road_detector.launch.py
```

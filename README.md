# CS492_Intelligence_Contest
#### forked from Deep Grasping ROS (Seunghyeok Back)

- ROS wrapper for DNN based robotic grasping algorithms
- Support Contact-GraspNet [[paper]](https://arxiv.org/abs/2103.14127) [[code]](https://github.com/NVlabs/contact_graspnet)

## TODO
- Check the execution environment and description `/doc`
- Check the operation of each module
- Create launch file
- Check the Grasp pose generation in Real Robot
- arm camera link.... (https://github.com/SeungBack/azure_kinect_manager check!)

## Contact-GraspNet Framework
```
Deep_grasping/src

Contact_Graspnet_server.py (for ROS)
/Contact_graspnet
 - Contact_graspnet_client.py (for ROS)
/uoais
 - launch/uoais_rs_d435.launch (need to modify name space)
 -(TOBO) uoais_client.py
(TODO)namespace 
```
## Env setup
### 1. Install Repository (CS492_Intelligence_contest/src) 
1. git clone 'CS_492_intelligence_contest' or 'gist-ailab/deep-grasping' 
2. 'cd src', git clone 'contact_graspnet','uoais'
```
git clone https://github.com/SeungBack/contact_graspnet.git
git clone https://github.com/gist-ailab/uoais.git
```
  + Enviroment setup(1~7) + Test "ROS nodes 1.Realsense D-435" ..(나중에 namespace 변경 필요할듯)
- (https://github.com/gist-ailab/uoais#environment-setup)
3. Install `easy_tcp_python2_3`, `open3d-ros-helper` (Use conda env!!!)
- https://github.com/SeungBack/open3d-ros-helper (env py2.7, pip2 install~)
- https://github.com/SeungBack/easy_tcp_python2_3 (env py2.7 or 3.6~, pip install~)


### 2. Create conda enviroment
1. contact_graspnet_env(for contact_graspnet_client.py)
- python 3.7, tensorflow 2.2, CUDA 10.1, CUCNN 7.6.0
- yml file location: https://github.com/SeungBack/contact_graspnet 
- Install : easy_tcp_python2_3

```
conda env create -f contact_graspnet_env.yml
pip install pickle-compat
pip install opencv-python
pip install easy-tcp-python2-3
```


2. uoais(for uoais.py : 작동 O)
 - Enviroment: python 3.7, pytorch 1.8.0+cu111, torchvision 0.9.0+cu111, CUDA 11.1, detectron2 v0.6+cu111+torch 1.8.0
```
conda create -n uoais python=3.7
conda activate uoais
pip install torch==1.8.0+cu111 torchvision==0.9.0+cu111 torchaudio==0.8.0 -f https://download.pytorch.org/whl/torch_stable.html
pip install shapely torchfile opencv-python pyfastnoisesimd rapidfuzz
``` 
(please CUDA version check: nvcc -V, torch.version.cuda ) 
(If, CUDA version confilct => uoais setup.py build develop 실행 안됨 )
    
- install detectron2...
 (https://detectron2.readthedocs.io/en/latest/tutorials/install.html#requirements)



## RUN
### Robot(HAETAE)
* If you run in CPU machine(attaced Haetae..), try http://192.168.10.10:11311 and wifi off

Haetae RUN(+ wrist_camera)
```
export ROS_MASTER_URI=http://192.168.0.100:11311
sudo route add -net 192.168.10.10 netmask 255.255.255.255 gw 192.168.0.100
(or 'robot')
(navigation_ws(janghyuk/12_ob..detec..)
source ./robot.sh real manip 
roslaunch pick_and_place_demo demo.launch sim:=false perception_src:=none
```

(To modify...)
``` 
export ROS_MASTER_URI=http://192.168.0.100:11311
sudo route add -net 192.168.10.10 netmask 255.255.255.255 gw 192.168.0.100
rostopic list

TODO: localhost:11311 -> haetae 
launch file 
bash file ....
```

- RViz(Haetae ROS 연결 확인 후)
```
export ROS_MASTER_URI=http://192.168.0.100:11311
source navigation_ws/devel/setup.bash
rosrun rviz rviz
```

### Contact graspnet server
- python == 2.7, with dependencies of rirolab/navigation_ws 
- master node 11311 필요(Haetae ROS 확인 후 돌려볼 것.:OK but we have to connect Haetae - uoais)
```
roscd deep_grasping && python src/contact_grasp_server.py
```

### Contact graspnet client
#### **1. contact graps client.py**
- python == 3.7 please create conda env(OK)
```
conda activate contact_graspnet_env \
    && roscd deep_grasping_ros/src/contact_graspnet \
    && CUDA_VISIBLE_DEVICES=0 python contact_graspnet/contact_grasp_client.py --local_regions --filter_grasps
```

#### **2. uoais client.py (예제는 작동 됨)**
```
conda activate uoais \
    && roscd deep_grasping/src/uoais \
    && CUDA_VISIBLE_DEVICES=1 python demo/uoais_client.py
```
We must make 'uoais_client.py' or other files \


**2.1 For uoais example 1 (Image example)**
```
python tools/run_on_OSD.py --use-cgnet --dataset-path ./sample_data --config-file configs/R50_rgbdconcat_mlc_occatmask_hom_concat.yaml
```
**2.2 For uoais example 2 (use ROS RViz)** \
We use ROS1 version + python3:: cv_bridge Problem \
If you make catkin_ws 'catkin_make', You need to create New workspace 
 
**Create 'cvbridge_build_ws [Link](https://cyaninfinite.com/ros-cv-bridge-with-python-3/)** 
```
sudo apt-get install python3-pip python-catkin-tools python3-dev python3-numpy
sudo pip3 install rospkg catkin_pkg
```
```
mkdir -p ~/cvbridge_build_ws/src

catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
# Instruct catkin to install built packages into install place. It is $CATKIN_WORKSPACE/install folder
catkin config --install
```
```
cd ~/cvbridge_build_ws/src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git


apt-cache show ros-melodic-cv-bridge | grep Version
    Version: 1.13.0-0xenial-20180416-143935-0800
# Checkout right version in git repo. In our case it is 1.13.0
cd src/vision_opencv/
git checkout 1.13.0
cd ../../

# Build
catkin build cv_bridge

# Extend environment with new package
source install/setup.bash --extend
```
- Check your workspace
```
python
from cv_bridge.boost.cv_bridge_boost import getCvType 
exit()
```

**Then, We can use this launch file**
```
conda activate uoais
roslaunch uoais uoais_rs_d435.launch 
```

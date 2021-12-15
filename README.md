
# CS492_Intelligence_Contest
#### forked from Deep Grasping ROS (Seunghyeok Back)

- ROS wrapper for DNN based robotic grasping algorithms
- Support Contact-GraspNet [[paper]](https://arxiv.org/abs/2103.14127) [[code]](https://github.com/NVlabs/contact_graspnet)

## Contact-GraspNet Framework
```
Deep_grasping/src

Contact_Graspnet_server.py (for ROS)
/Contact_graspnet
 - Contact_graspnet_client.py (for ROS)
/uoais
 - launch/uoais_rs_d435.launch (need to modify name space)
 
```
## Env setup
### 1. Install Repository (CS492_Intelligence_contest/src) 
1. git clone 'CS_492_intelligence_contest' or 'gist-ailab/deep-grasping' 
2. 'cd src', git clone 'contact_graspnet','uoais'
```
git clone https://github.com/SeungBack/contact_graspnet.git
git clone https://github.com/gist-ailab/uoais.git
```
  + Enviroment setup(1~7) + Test ROS nodes 1.Realsense D-435 ..(나중에 namespace 변경 필요할듯)
- (https://github.com/gist-ailab/uoais#environment-setup)
3. Install `easy_tcp_python2_3`, `open3d-ros-helper` (Use conda env!!!)
- https://github.com/SeungBack/open3d-ros-helper (env py2.7, pip2 install~)
- https://github.com/SeungBack/easy_tcp_python2_3 (env py2.7 or 3.6~, pip install~)


### 2. Create conda enviroment
1. contact_graspnet_env(for contact_graspnet_client.py)
- python 3.7, tensorflow 2.2, CUDA 10.1, CUCNN 7.6.0
- yml file location: https://github.com/SeungBack/contact_graspnet 
- Install : easy_tcp_python2_3, 

```
conda env create -f contact_graspnet_env.yml
pip install pickle-compat
pip install opencv-python
pip install easy-tcp-python2-3
```


2. uoais(for uoais.py)
 - python 3.7, pytorch 1.9.0, torchvision 0.9.0, CUDA 11.1(or 10.1), detectron2 v0.5 or v0.6
```
conda create -n uoais python=3.7
conda activate uoais
pip install torch torchvision
pip install shapely torchfile opencv-python pyfastnoisesimd rapidfuzz
``` 

- install detectron2...
 (https://detectron2.readthedocs.io/en/latest/tutorials/install.html#requirements)

```
python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'
``` 
(add --user if you don't have permission)

Or, to install it from a local clone:
```git clone https://github.com/facebookresearch/detectron2.git
python -m pip install -e detectron2
```

## RUN
### Robot
- Use Haetae2025. If you use RViz another machine, you have to run 'pick_and_place_demo/demo.launch' on haetae2025
``` 
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_HOST_URI=http://192.168.0.24 ??????
sudo route add -net 192.168.10.10 netmask 255.255.255.255 gw 192.168.0.100

source ./robot.sh real manip (or source ./robot_zsh.sh real manip)
roslaunch pick_and_place_demo demo.launch sim:=false perception_src:=none
```

### RViz
- If you want to use 'wrist_camera' topics, have to run 'demo.launch'
```
export ROS_MASTER_URI=http://192.168.0.100:11311
source navigation_ws/devel/setup.bash
rosrun rviz rviz
```

### Contact graspnet server
- python == 2.7, with dependencies of rirolab/navigation_ws 
```
roscd deep_grasping && python src/contact_grasp_server.py
```


### Contact graspnet client
1. contact graps client.py
- python == 3.7 please create conda env
```
conda activate contact_graspnet_env \
    && roscd deep_grasping/src/contact_graspnet \
    && CUDA_VISIBLE_DEVICES=0 python contact_graspnet/contact_grasp_client.py --local_regions --filter_grasps
```

2. uoais client.py
```
conda activate uoais \
    && roscd deep_grasping/src/uoais \
    && CUDA_VISIBLE_DEVICES=1 python demo/uoais_client.py
```

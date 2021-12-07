
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

### Contact-GraspNet
```
Deep_grasping/src

Contact_Graspnet_server.py (for ROS)
/Contact_graspnet
 - Contact_graspnet_client.py (for ROS)
/uoais
 - launch/uoais_rs_d435.launch (need to modify name space)
 
```
### Env setup
#### Install Repository (CS492_Intelligence_contest/src) 
1. git clone `CS_492_intelligence_contest` or `gist-ailab/deep-grasping` 
2. `cd src`, git clone `contact_graspnet`,`uoais`
`git clone https://github.com/SeungBack/contact_graspnet.git`
`git clone https://github.com/gist-ailab/uoais.git
    ` + Enviroment setup(1~7) + Test ROS nodes 1.Realsense D-435 ..(나중에 namespace 변경 필요할듯)
      (https://github.com/gist-ailab/uoais#environment-setup)
3. Install `easy_tcp_python2_3`, `open3d-ros-helper` (Use conda env!!!)
 https://github.com/SeungBack/open3d-ros-helper (env py2.7, pip2 install~)
 https://github.com/SeungBack/easy_tcp_python2_3 (env py2.7 or 3.6~, pip install~)


#### Create conda enviroment
1. contact_graspnet_env(for contact_graspnet_client.py)
- python 3.7, tensorflow 2.2, CUDA 10.1, CUCNN 7.6.0
- yml file location: https://github.com/SeungBack/contact_graspnet 
`conda env create -f contact_graspnet_env.yml`


2. uoais(for uoais.py)
 - python 3.7, pytorch 1.9.0, torchvision 0.9.0, CUDA 11.1(or 10.1), detectron2 v0.5 or v0.6
`conda create -n uoais python=3.7
conda activate uoais
pip install torch torchvision
pip install shapely torchfile opencv-python pyfastnoisesimd rapidfuzz` 

- install detectron2...
 (https://detectron2.readthedocs.io/en/latest/tutorials/install.html#requirements)

`python -m pip install 'git+https://github.com/facebookresearch/detectron2.git'` 
(add --user if you don't have permission)

Or, to install it from a local clone:
```git clone https://github.com/facebookresearch/detectron2.git
python -m pip install -e detectron2```

### RUN
Robot
``` 
TODO: localhost:11311 -> haetae 
launch file 
bash file ....
```

Contact graspnet server
- python == 2.7, with dependencies of rirolab/navigation_ws 
```
roscd deep_grasping && python src/contact_grasp_server.py
```

Contact graspnet client
1. contact graps client.py
- python == 3.7 please create conda env
```
conda activate contact_graspnet_env \
    && roscd deep_grasping_ros/src/contact_graspnet \
    && CUDA_VISIBLE_DEVICES=0 python contact_graspnet/contact_grasp_client.py --local_regions --filter_grasps
```
2. uoais client.py
```
conda activate uoais \
    && roscd deep_grasping_ros/src/uoais \
    && CUDA_VISIBLE_DEVICES=1 python demo/uoais_client.py
```

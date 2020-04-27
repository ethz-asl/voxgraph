# Voxgraph
![voxgraph_logo_github](https://user-images.githubusercontent.com/6238939/79927519-434b4080-8440-11ea-9187-92e28466035b.png)

Voxgraph is a globally consistent volumetric mapping framework. It represents the world as a collection of Signed Distance Function submaps, which it aligns through pose graph optimization. Each submap is based on [voxblox](https://github.com/ethz-asl/voxblox) and the collection itself is managed using [c-blox](https://github.com/ethz-asl/cblox).

Local consistency is maintained through registration constraints between overlapping submap pairs. The computational cost of these constraints is significantly reduced by directly exploiting the underlying Signed Distance Function representation of the submaps. This makes it possible to globally optimize maps with hundreds of overlapping submap pair constraints, in a matter of seconds even on computationally constrained platforms.

Global consistency can be achieved by adding absolute pose or loop closure constraints to the pose graph. We expect to provide an internal mechanism in voxgraph to detect global loop closures in the future. In the meantime, an interface to add loop closures from external sources is provided through a ROS topic. Absolute pose constraints are mostly implemented, but this work has not yet been completed since we don't have a direct use-case for them. Please create a [GitHub issue](https://github.com/ethz-asl/voxgraph/issues) if you would like to use them. Of course, [pull requests](https://github.com/ethz-asl/voxgraph/pulls) are even more welcome.

## Paper and Video
[![Video player thumbnail for GitHub](https://user-images.githubusercontent.com/6238939/80018022-b94db700-84d5-11ea-9118-9540f7cdd67b.JPG)](https://youtu.be/K2HPAMqvh3E)

When using voxgraph for research, please cite the following paper [[preprint](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/385682/Voxgraph-ETHpreprintversion.pdf?sequence=1&isAllowed=y) | [IEEE](https://ieeexplore.ieee.org/document/8903279)]:

```
@ARTICLE{reijgwart2020voxgraph,
author={V. {Reijgwart} and A. {Millane} and H. {Oleynikova} and R. {Siegwart} and C. {Cadena} and J. {Nieto}},
journal={IEEE Robotics and Automation Letters},
title={Voxgraph: Globally Consistent, Volumetric Mapping Using Signed Distance Function Submaps},
year={2020}}
```

<details>
<summary>Abstract</summary>
<br>
Globally consistent dense maps are a key requirement for long-term robot navigation in complex environments. While previous works have addressed the challenges of dense mapping and global consistency, most require more computational resources than may be available on-board small robots. We propose a framework that creates globally consistent volumetric maps on a CPU and is lightweight enough to run on computationally constrained platforms. Our approach represents the environment as a collection of overlapping signed distance function (SDF) submaps and maintains global consistency by computing an optimal alignment of the submap collection. By exploiting the underlying SDF representation, we generate correspondence-free constraints between submap pairs that are computationally efficient enough to optimize the global problem each time a new submap is added. We deploy the proposed system on a hexacopter micro aerial vehicle (MAV) with an Intel i7-8650 U CPU in two realistic scenarios: mapping a large-scale area using a 3D LiDAR and mapping an industrial space using an RGB-D camera. In the large-scale outdoor experiments, the system optimizes a 120 × 80 m map in less than 4 s and produces absolute trajectory RMSEs of less than 1 m over 400 m trajectories. Our complete system, called voxgraph, is available as open-source.
</details>

## Install
Voxgraph extends voxblox and runs on [ROS](https://www.ros.org/). Therefore start by installing and configuring these as described in the [voxblox installation instructions](https://voxblox.readthedocs.io/en/latest/pages/Installation.html).

Next, clone this repository and its dependencies into your catkin workspace. 
If you have [SSH keys for GitHub](https://help.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh) set up:
```shell script
cd ~/catkin_ws/src
git clone git@github.com:ethz-asl/voxgraph.git

wstool merge ./voxgraph/voxgraph_ssh.rosinstall
wstool update
```
<details>
<summary>No ssh keys?</summary>
<br>

```shell script
cd ~/catkin_ws/src/
git clone https://github.com/ethz-asl/voxgraph.git

wstool merge ./voxgraph/voxgraph_https.rosinstall
wstool update
```
</details>

Then compile voxgraph and its dependencies with:
```shell script
cd ~/catkin_ws/src/
catkin build voxgraph
```
    
## Run
##### Demo
We provide a demo dataset [here](http://robotics.ethz.ch/~asl-datasets/2020_voxgraph_arche/arche_flight1_2ms_indoor-outdoor-figure-8.zip), which features a hexacopter flying through an indoor-outdoor search and rescue training site. The rosbag includes the robot's visual, inertial and LiDAR sensor data, and the GPS RTK measurements used for the evaluations in the paper.

Running this example requires a couple more packages than the basic install for performing lidar undistortion. These are clone and build these using:
```shell script
cd ~/catkin_ws/src/
wstool merge ./voxgraph/arche_ssh.rosinstall
wstool update
catkin build lidar_undistortion
```
<details>
<summary>No ssh keys?</summary>
<br>

```shell script
cd ~/catkin_ws/src/
wstool merge ./voxgraph/arche_https.rosinstall
wstool update
catkin build lidar_undistortion
```
</details>

Download and unzip the dataset
```shell script
wget -P ~/Downloads http://robotics.ethz.ch/~asl-datasets/2020_voxgraph_arche/arche_flight1_2ms_indoor-outdoor-figure-8.zip
unzip ~/Downloads/arche_flight1_2ms_indoor-outdoor-figure-8.zip
```
Launch the example
```shell script
roslaunch voxgraph arche_demo.launch rosbag_path:=${HOME}/Downloads/arche_flight1_2ms_indoor-outdoor-figure-8.bag
```
If everything has worked correctly this should open rviz and within a few seconds the first fragment of the map should appear.

Note that visualizing the whole mesh can be heavy, particularly as the map grows. If you are on a weak computer, consider disabling the mesh visualization once you have confirmed voxgraph is running correctly (uncheck the box next to "Merged Map" in rviz). The current state of the map can be saved at any time using a service call:
```shell script
rosservice call /voxgraph_mapper/save_combined_mesh "file_path: '$HOME/mesh.ply'"
```
This mesh is viewable through a number of programs. One simple example is [meshlab](http://www.meshlab.net/) such that the mesh is viewed
```shell script
meshlab ~/mesh.ply
```

##### Your own dataset
The basic requirements for running voxgraph are:
 1. an odometry source, and
 2. a source of dense (depth) data.

The system is agnostic as to where the odometry comes from. In the demo [above](#demo) we used visual odometry provided by [rovio](https://github.com/ethz-asl/rovio), however, we have also run voxgraph using diverse odometry sources, for example leg odometry. The requirement is that there a link on the tf tree between the `input_odom_frame` and `input_base_link_frame`. These frame names are configurable as rosparams. They are set, for example, for the demo in the parameter file [here](https://github.com/ethz-asl/voxgraph/blob/feature/release/voxgraph/config/voxgraph_mapper.yaml#L9). 

For depth data we have tried RGB-D and 3D LiDAR sensors. The input datastream is provided to the system as rostopic with a configurable name through the rosparam `pointcloud_topic`. See [here](https://github.com/ethz-asl/voxgraph/blob/feature/document_example/voxgraph/launch/arche_demo.launch#L6) for the topic used in the above example.

Providing these two topics should be enough to get you started. 

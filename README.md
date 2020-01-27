# Voxgraph
_[Logo - Coming soon]_

Voxgraph is a globally consistent volumetric mapping framework. It represents the world as a collection of Signed Distance Function submaps, each based on [voxblox](https://github.com/ethz-asl/voxblox). These are then aligned through pose graph optimization.

Local consistency is maintained through registration constraints between overlapping submap pairs. The computational cost of these constraints is significantly reduced by directly exploiting the underlying Signed Distance Function representation of the submaps. This makes it possible to globally optimize maps with hundreds of overlapping submap pair constraints, in a matter of seconds even on computationally constrained platforms.

Global consistency can be achieved by adding absolute pose or loop closure constraints to the pose graph. We expect to provide an internal mechanism in voxgraph to detect global loop closures in the future. In the meantime, an interface to add loop closures from external sources is provided through a ROS topic. Absolute pose constraints are mostly implemented, but this work has not yet been completed since we don't have a direct use-case for them. Please create a [GitHub issue](https://github.com/ethz-asl/voxgraph/issues) if you would like to use them. Of course, [pull requests](https://github.com/ethz-asl/voxgraph/pulls) are even more welcome.

## Paper and Video
_[Video - Coming soon]_

When using voxgraph for research, please cite the following paper [[preprint](https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/385682/Voxgraph-ETHpreprintversion.pdf?sequence=1&isAllowed=y)|[IEEE](https://ieeexplore.ieee.org/document/8903279)]:

```
@ARTICLE{reijgwart2020voxgraph,
author={V. {Reijgwart} and A. {Millane} and H. {Oleynikova} and R. {Siegwart} and C. {Cadena} and J. {Nieto}},
journal={IEEE Robotics and Automation Letters},
title={Voxgraph: Globally Consistent, Volumetric Mapping Using Signed Distance Function Submaps},
year={2020}}
```

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
Otherwise:
```shell script
cd ~/catkin_ws/src/
git clone https://github.com/ethz-asl/voxgraph.git

wstool merge ./voxgraph/voxgraph_https.rosinstall
wstool update
```


Then compile voxgraph and its dependencies with:
```shell script
cd ~/catkin_ws/src/
catkin build voxgraph
```
    
## Run
##### Demo
_[Coming soon]_

##### Your own dataset
_[Coming soon]_

## TODOs
- [ ] Remove dependency on maplab_msgs

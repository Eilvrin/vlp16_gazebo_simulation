# Instructions

## Overview

### velodyne16_gazebo
 * Controller for the vlp16 in gazebo.
 * Spherical projections of assembled point clouds and export to .h5 data.
 * Multiple helper nodes and launch files for different setups.

## Installation

### Install dependencies
        sudo apt-get install libhdf5-dev libhdf5-serial-dev

### Troubleshooting

#### Compile error: cannot find lhdf5
        cd /usr/lib/x86_64-linux-gnu
        sudo ln -s libhdf5_serial.so.10.1.0 libhdf5.so

#### Compile error: Undefined reference to symbol
```
mkdir build; cd build
cmake .. -DCMAKE_CXX_COMPILER=mpic++
make
```

### velodyne_simulator
Model of vlp16 for gazebo, outputs single scans.

## Detailed information

#### Laboratory model
Extract [models.tar.gz](https://drive.google.com/open?id=1OpUiD1Df3Uxj6SkSCFRYlGGnHzDrKc_W) into `.gazebo/models`.
#### Ros packages
##### Velodyne_simulator
Package `velodyne_simulator` contains VLP-16 description and plugin for laser scans. This is a modified version of https://bitbucket.org/DataspeedInc/velodyne_simulator.   
In `velodyne_simulator/velodyne_description/urdf/VLP-16.urdf.xacro` the following parameters should be changed depending on the building models for simulation.

For laboratory 080 building:  
```html
   <collision name="${name}_base_link_collision">
     <origin rpy="0 0 0" xyz="0 0 0.03585"/>
     <geometry>
       <!-- <cylinder radius="0.0516" length="0.0717"/> -->
       <sphere radius="0.3"/>
     </geometry>
   </collision>
   ...
   <range>
     <min>0.31</min>
     <!-- <min>0.055</min> -->
     <max>140.0</max>
     <resolution>0.001</resolution>
   </range>
```

 For SUNCG dataset:  
```html
    <collision name="${name}_base_link_collision">
      <origin rpy="0 0 0" xyz="0 0 0.03585"/>
      <geometry>
        <cylinder radius="0.0516" length="0.0717"/>
        <!--<sphere radius="0.3"/>-->
      </geometry>
    </collision>
    ...
    <range>
      <min>0.31</min>
      <min>0.055</min>
      <!-- <max>140.0</max> -->
      <resolution>0.001</resolution>
    </range>
```
##### Velodyne16_gazebo
Package `velodyne16_gazebo` contains the simulation of VLP-16 and projection to .h5 files.
In `velodyne16_gazebo/config/scan_to_image.yaml` the following parameters should be specified:
- `out_folder_path`: the path to folder where the resulting files will be stored
- `suncg_house`: whether the simulation is performed on SUNCG dataset or on laboratory 080 building (`true` - SUNCG dataset, `false` - laboratory 080 building)

To launch the simulation on laboratory 080 building:
```
roslaunch velodyne16_gazebo velodyne16_gazebo.launch
```
To launch the simulation on SUNCG dataset:
```
roslaunch velodyne16_gazebo velodyne16_gazebo.launch world_name:=<path>/house.world
```
The results are the simulated .pcd clouds, .h5 files of projected images, and .txt files containing the transforms to the fixed frame.

###### Additional nodes
- To launch just the projection of assembled clouds from topic to files (without simulation of assembled clouds, for example to project the recorded point clouds from Smart Walker to .h5 files):
```
roslaunch velodyne16_gazebo scan_to_image.launch
```
The output files will be saved according to `out_folder_path` parameter in  `velodyne16_gazebo/config/scan_to_image.yaml`.
- To project the assembled clouds from .pcd files.
In the `velodyne16_gazebo/launch/pcd_to_image.launch` specify `<param name="path_to_clouds" type="string" value="<full_path_to_folder_with_pcd_clouds>"/>`
```
roslaunch velodyne16_gazebo pcd_to_image.launch
```
The output files will be saved according to `out_folder_path` parameter in  `velodyne16_gazebo/config/scan_to_image.yaml`.
- To project the predictions of the neural network back to .pcd clouds. Change the following parameters in `velodyne16_gazebo/config/image_to_pcd.yaml`:
  - `path_to_pcdlist`: path to .txt file, which contains the list of full paths to .pcd files
  - `path_to_h5`: path to folder, containing the predictions of neural network in .h5 format
  - `out_path_pcd`: path to folder where to save the resulting .pcd files with labels      
```
roslaunch velodyne16_gazebo image_to_pcd.yaml
```

# pc_downsampling

## Installation 
### Build locally
Requirements:
* ROS
* PCL

```bash
cd ~/catkin_ws/src
git clone https://github.com/ozakiryota/pc_downsampling.git
cd ..
catkin_make
```

### Build with Docker
```bash
git clone https://github.com/ozakiryota/pc_downsampling.git
cd pc_downsampling/docker
./build.sh
```

## Usage
1. Edit the launch file
1. Run
```bash
roslaunch pc_downsampling pc_voxelgrid_filter.launch
```
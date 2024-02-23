## Init submodule

```
git submodule init
git submodule update --remote
```

## How to build

```
cd to your ros workspace
colcon build --packages-select turtlebot3_tourguide
```

# How to run

```
source install/local_setup.sh
ros2 run turtlebot3_tourguide turtlebot3_tourguide
```
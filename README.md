
# Explo-Ros

  

#### Prerequisites

  

You need to have NodeJS and npm installed with the global following packages :

```

npm i --globally typescript

```

  

The package is composed of two ROS nodes :

- CouchManager : This node manage all the interactions between ROS and CouchDB. To Configure the DataBase, you need to configure the *"couchHosts.json"* and "*couchManagerConf.json*" files.

- Agents : The Agents node does all the exploration of a given grid/environnment (only of grids for now).

### Topics

This rclnodejs project communicates with 3 topics : 

| Name | Publishers | &rarr;| Subscribers  | Message type | 
|--|--|--|--|--|
| replication_changes | CouchManager | &rarr; | Agents | node_pkg/msg/MapUpdate |
| update_map | Agents | &rarr;|CouchManager | node_pkg/msg/MapUpdate |
| update_pos | Agents | &rarr;|CouchManager | geometry_msgs/msg/Point |

The *replication_changes* topic sends change informations from the other databases from the cluster.
The messages flowing through this topic only comes from distant hosts. The changes in the local DataBase are not sent inside this topic.

The *update_map* topic have information about the "real" world sent by the agent. It is all it sees that is sent inside this topic for the DB to save it and send the info to the other hosts.

*update_pos* sends the Agent positition to the CouchManager so the coordinates of the exploring agent is shared through the cluster.

IMPORTANT : For the topics to work correctly, you need to have a ROS2 package called *node_pkg* defining the following messages types :

```
# /Cell.msg
int32 x
int32 y
int8 value
```
Info : The value of a cell is an int between 0 and 100. It describes the probability of a wall being there. Thus, 100 is a wall, 0 is empty and 50 is unknown. 
```
# /MapUpdate.msg
Cell[] cells
```
If you don't have them, you need to create a ros2 package inside your ros2 workspace with the new types:
```
ros2 pkg create --build-type ament_cmake node_pkg
mkdir node_pkg/msg
cd node_pkg/msg
echo "int32 x\n int32 y \n int8 value" > ./Cell.msg
echo "Cell[] cells" > ./MapUpdate.msg
```

You also need to reference them inside your *CMakeLists.txt*.
You need these lines inside of it before the build part:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cell.msg"
  "msg/MapUpdate.msg"
)
```

You will also need these lines inside your *package.xml* :
```
<member_of_group>rosidl_interface_packages</member_of_group>
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```
Once you have all of this, there is an example of usage inside *app.ts*. To run it, just use the following npm commands at the root of the Explo-Ros project:
```
npm run build
npm start
```

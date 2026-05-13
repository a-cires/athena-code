# CAN Node for UMD Loop
Exposes a can_node executable that creates 2 topics, /can_tx and /can_rx
If you publish a umdloop_theseus_can_messages/msg/CANA or a umdloop_theseus_can_messages/msg/CANB message to "/can_tx", the can_node from this package will send that CAN message over the CAN bus. This makes it really easy to send CAN messages through ROS.
Both message types (interfaces) are custom. Their format is detailed below:
#### CANA
```
uint16 id
uint8[] data
```

#### CANB
```
uint32 id
uint8[] data
```

As you can see, these messages are not complex. The can_node abstracts away how CAN messages are handled on the back end. You would mainly work with the /can_tx and /can_rx topics
\\ TO-DO Add in protections against incorrect message inputs, such as invalid arbitration ids

## Installation/Use
### Python
#### In your code
You would want to publish to the /can_tx topic or subscribe to the /can_rx topic, but you still need to use the custom message interfaces to do so
In the code that needs it, import either CANA or CANB like so:
```
from umdloop_theseus_can_messages.msg import CANA # or CANB
```

#### Launch Files
Add a node to your launch description which includes the executable from this package like so:
```
Node(
package="umdloop_can",
namespace="whatever_you_want_here",
executable="can_node",
)
```
// TO-DO add in support for namespace declaration

#### package.xml
```
<exec_depend>umdloop_can</exec_depend>
<exec_depend>umdloop_theseus_can_messages</exec_depend>
```

### C++
#### In your code
You will want to publish to the /can_tx topic or subscribe to the /can_rx topic, but you still need to use the custom message interface to do so
In the code that needs it, import either CANA or CANB like so:
```
# include umdloop_theseus_can_messages/msg/CANA // or CANB
```

#### CMakeLists.txt
```
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(umdloop_theseus_can_messages REQUIRED)

add_executable(some_node src/node_code_file.cpp)
ament_target_dependencies(some_node rclcpp umdloop_theseus_can_messages)

install(TARGETS
  some_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

#### package.xml
```
<depend>umdloop_theseus_can_messages</depend>
```

### General requirements
// TO-DO add in rosdep functionality to make this easier for the people after me
You're going to need to install the following library with pip:
```
pip install python-can
```

## Sanity Check
1. Set up your workspace. From your ros_ws root:
```
source /opt/ros/humble/setup.bash
colcon build --packages-select umdloop_theseus_can_messages umdloop_can
source install/local_setup.bash
```

2. Spin up a can_node node!
```
ros2 run umdloop_can can_node
```

You should start to see your terminal fill up with messages. You can also open up a new termainal and double check against candump:
```
candump can1
```
This will show you all of the CAN messages on your CAN network device


You can publish directly to the /can_tx topic like so:
```
ros2 topic pub /can_tx /umdloop_theseus_can_messages/msg/CANA "{id: 321, data: [162, 0, 0, 0, 40, 35, 0, 0]}"
```
In case you need to troubleshoot some things.

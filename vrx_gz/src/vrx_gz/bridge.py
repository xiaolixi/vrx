from dataclasses import dataclass
from enum import Enum

## Bridge Direction 双向的、gazebo2ros ros2gazebo
class BridgeDirection(Enum):
    BIDIRECTIONAL = 0
    GZ_TO_ROS = 1
    ROS_TO_GZ = 2

## Bridge Direction Symbols dict
DIRECTION_SYMS = {
    BridgeDirection.BIDIRECTIONAL: '@',
    BridgeDirection.GZ_TO_ROS: '[',
    BridgeDirection.ROS_TO_GZ: ']',
}

## Bridge class
## 用于表示gazebo和ros之间的bridge
## topic是依据gz_topic为准，gz转到ros是gz，ros转到ros，要求ros的topic就是gz_topic
@dataclass
class Bridge:
    gz_topic: str  # gazebo topic name
    ros_topic: str  # ros topic name
    gz_type: str  # gazebo topic type
    ros_type: str  # ros topic type
    direction: BridgeDirection  # bridge direction

    def argument(self): # 生成bridge argument str
        out = f'{self.gz_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.gz_type}'
        return out

    def remapping(self): # 生成remapping tuple (gz_topic, ros_topic)
        return (self.gz_topic, self.ros_topic)

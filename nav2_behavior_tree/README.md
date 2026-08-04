# nav2_behavior_tree

This module is used by the nav2_bt_navigator to implement a ROS2 node that executes navigation Behavior Trees for either navigation or autonomy systems. The nav2_behavior_tree module uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) for the core Behavior Tree processing.

The nav2_behavior_tree module provides:
* A C++ template class for easily integrating ROS2 actions and services into Behavior Trees,
* Navigation-specific behavior tree nodes, and
* a generic BehaviorTreeEngine class that simplifies the integration of BT processing into ROS2 nodes for navigation or higher-level autonomy applications.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-bt-xml.html) for additional parameter descriptions and a list of XML nodes made available in this package. Also review the [Nav2 Behavior Tree Explanation](https://navigation.ros.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package. A [tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html) is also provided to explain how to create a simple BT plugin.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins. 

## The bt_action_node Template and the Behavior Tree Engine

The [bt_action_node template](include/nav2_behavior_tree/bt_action_node.hpp) allows one to easily integrate a ROS2 action into a BehaviorTree. To do so, one derives from the BtActionNode template, providing the action message type. For example,

```C++
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
    ...
};
```

The resulting node must be registered with the factory in the Behavior Tree engine in order to be available for use in Behavior Trees executed by this engine.

```C++
BehaviorTreeEngine::BehaviorTreeEngine()
{
    ...

  factory_.registerNodeType<nav2_behavior_tree::FollowPathAction>("FollowPath");

    ...
}
```

Once a new node is registered with the factory, it is now available to the BehaviorTreeEngine and can be used in Behavior Trees. For example, the following simple XML description of a BT shows the FollowPath node in use:

```XML
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <ComputePathToPose goal="${goal}"/>
      <FollowPath path="${path}" controller_property="FollowPath"/>
    </Sequence>
  </BehaviorTree>
</root>
```
The BehaviorTree engine has a run method that accepts an XML description of a BT for execution:

```C++
  BtStatus run(
    BT::Blackboard::Ptr & blackboard,
    const std::string & behavior_tree_xml,
    std::function<void()> onLoop,
    std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));
```

See the code in the [BT Navigator](../nav2_bt_navigator/src/bt_navigator.cpp) for an example usage of the BehaviorTreeEngine.

For more information about the behavior tree nodes that are available in the default BehaviorTreeCPP library, see documentation here: https://www.behaviortree.dev/docs/3.8/learn-the-basics/BT_basics

## 中文翻译

# nav2_behavior_tree

该模块被 nav2_bt_navigator 使用，用于实现执行导航或自主系统行为树的 ROS 2 节点。核心行为树处理由 Behavior-Tree.CPP 库提供。

本包提供三类能力：把 ROS 2 Action 和 Service 集成到行为树的 C++ 模板类；Nav2 专用行为树节点；以及简化行为树处理并方便集成到 ROS 2 导航或上层自主节点的 BehaviorTreeEngine。配置指南、默认树说明、可用 XML 节点和新建 BT 插件教程仍以原文链接为准。

## bt_action_node 模板与行为树引擎

bt_action_node.hpp 中的模板允许开发者通过派生 BtActionNode，把指定的 ROS 2 Action 消息类型接入行为树。新节点需要在 BehaviorTreeEngine 工厂中注册，注册后才能在行为树 XML 中使用。原文示例展示了 FollowPath 节点注册、ComputePathToPose 与 FollowPath 的顺序执行，以及 BehaviorTreeEngine::run 的黑板、XML、循环回调、取消回调和循环超时参数。

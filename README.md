# Traveler Information Message (TIM)
- Ubuntu 22.04
- ROS2 Humble

``` mermaid
graph LR
subgraph Infra
    OBU
end
subgraph Vehicle
    RSU
end
OBU -->|tim| RSU

rviz((rviz))
OBU -.->|infra_markers| rviz
RSU -.->|vehicle_markers| rviz
```
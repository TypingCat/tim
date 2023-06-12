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
OBU -->|TIM| RSU

rviz((rviz))
OBU & RSU -.->|marker| rviz
```
# Traveler Information Message (TIM)
## 개요
- 프로그램종류코드: 시스템 S/W, 데이터 통신, 유무선 통신 프로토콜 지원 S/W
- 적용분야: 자율주행
- 주요내용
    - 본 프로그램의 특징: SAE International에서 배포하는 표준 J2735를 준수하는 데이터 타입 Extended TIM을 정의한 라이브러리와, 그 검증을 위한 시뮬레이션 프로그램
    - 주요기능: TIM 생성, string 변환(for UDP), rosmsg/rviz 변환(for ROS)
    - 사용방법: 빌드 후 생성되는 라이브러리 파일 libTIM.so 활용
    - 판매구분: 비상업용
- 사용 OS: Ubuntu 22.04
- 사용기종: IBM-PC 호환기종
- 사용언어: C++17
- 필요한 프로그램: ROS2 Humble
- 규모: 압축 Byte
- 프로그램 형태: 소스파일, 오브젝트파일, 실행파일


## 라이브러리



## 검증 프로그램
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
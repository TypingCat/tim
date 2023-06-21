# Traveler Information Message (TIM)
## 개요
- 프로그램종류코드: 시스템 S/W, 데이터 통신, 유무선 통신 프로토콜 지원 S/W
- 적용분야: 자율주행
- 주요내용
    - 본 프로그램의 특징: SAE International에서 배포하는 표준 J2735를 준수하는 TIM을 확장 정의한 라이브러리와, 그 검증을 위한 시뮬레이션 프로그램
    - 주요기능: TIM 생성, string 변환 (for UDP), rosmsg/rviz 변환 (for ROS)
    - 사용방법: 빌드 후 생성되는 라이브러리 파일 libTIM.so 활용
    - 판매구분: 비상업용
- 사용 OS: Ubuntu 22.04
- 사용기종: IBM-PC 호환기종
- 사용언어: C++17
- 필요한 프로그램: ROS2 Humble
- 규모: 164,745 Byte
- 프로그램 형태: 소스파일, 오브젝트파일, 실행파일

---
## 라이브러리
### TIM 생성
빈 TIM을 생성한다. 인프라와 객체들의 정보를 담는다.
``` cpp
TIM::TIM();
```

ROS 메시지를 TIM으로 변환한다. ROS2 네트워크에서 TIM을 구독할 수 있다.
``` cpp
TIM::TIM(const tim::msg::TravelerInformationMessage & rosmsg);
```

문자열을 TIM으로 변환한다. UDP 통신을 위한 패킷 파싱으로 활용한다.
``` cpp
TIM::TIM(const std::string & s);
```

### TIM 변환
TIM을 ROS 메시지로 변환한다. ROS2 네트워크에 TIM을 발행할 수 있다.
``` cpp
tim::msg::TravelerInformationMessage TIM::to_rosmsg();
```

TIM을 ROS 마커로 변환한다. ROS2 rviz를 통해 TIM 객체들을 시각화할 수 있다.
``` cpp
visualization_msgs::msg::MarkerArray TIM::to_rviz();
```

TIM을 문자열로 변환한다. UDP 통신을 위한 객체 직렬화로 활용한다.
``` cpp
std::string TIM::to_string();
```

---
## 메시지 구조
- TIM: 자율주행 V2I 통신 용도로 확장한 SAE J2735 MSG_TravelerInformationMessage
    - msgCnt: 메시지 생성 순서
    - dataFrames: TIM 기본 프레임
        - notUsed
        - frameType: 프레임 유형
        - msgId: 메시지 ID
        - startTime: 메시지 생성 시각
        - durationTime: 메시지 지속 시각
        - priority: 표출 우선순위
        - notUsed1
        - regions: 앵커의 위도, 경도
        - notUsed2
        - notUsed3
    - regionals: 확장 구조, 엣지 인프라가 인지한 정보
        - timeStamp: 메시지 송신 시각
        - processingTime: 연산 소요 시간
        - edge: 엣지 인프라 정보
            - id: 엣지 인프라 ID
            - coordinateSystem: EPSG 기준 좌표계
            - pose: 엣지 인프라 좌표
        - objects: 엣지 인프라가 인지한 객체들 정보
            - id: 객체 ID
            - pose: 객체 자세
            - velocity: 객체 속도
            - footprint: 객체 형태
            - object: 객체 타입 분류
            - location: 객체 위치 분류
            - action: 객체 행동 분류
            - trajectoryForecasting: 객체 미래궤적 예상

---
## 검증 시뮬레이션
``` cmd
cd {ros2_humble_workspace}
colcon build
ros2 launch tim test.py
```

``` mermaid
graph LR
subgraph Infra_0: red
    observation(observation) -->|tim| OBU(OBU)
end
subgraph Vehicle_0: green
    RSU_0(RSU)
end
subgraph Vehicle_1: yellow
    RSU_1(RSU)
end
objects(objects) -.->|detection| observation
OBU -->|tim_rosmsg| RSU_0
OBU -->|tim_string| RSU_1

rviz(rviz)
OBU -.->|infra_markers| rviz
RSU_0 -.->|vehicle_markers| rviz
RSU_1 -.->|vehicle_markers| rviz
```

![test](https://github.com/TypingCat/tim/assets/16618451/8ec0bc88-f25f-449b-b235-38c3fe5f07c9)

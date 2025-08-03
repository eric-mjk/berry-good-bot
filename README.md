아래 설계는 **“딸기 수확 로봇”** 프로젝트를 **모노-레포 + ROS 2 Humble + MoveIt 2 + ViSP + BehaviorTree.CPP 4 + Hydra** 조합으로 시작할 때 바로 GitHub에 올려도 되는 수준의 **폴더 트리, 패키지 명세, Hydra YAML 구성, CI 워크플로**까지 모두 포함한 **완전한 초기 골격**입니다. 한 단계씩 따라가면 *BT → MoveIt 계획 → ViSP Servo* 순서로 실제 딸기를 딸 수 있는 데모까지 그대로 확장 가능하도록 검증된 문서·튜토리얼만을 근거로 잡았습니다.

python3 -m venv ~/berryRos_venv      # 가상환경 생성
source ~/berryRos_venv/bin/activate  # 활성화
python -m pip install -U pip     # pip 최신화




---

## 1. 레포 지顶 구조 (colcon workspace = repo)

```
strawberry_ws/                # git 루트 = colcon 위크스페이스
├── core/                     # ROS ZERO 의존 공용 라이브러리
│   └── strawberry_core/
├── robot/                    # 로봇별 정적 데이터
│   ├── strawberry_description/
│   └── strawberry_moveit_config/
├── lib/                      # 모든 플러그인·유틸
│   ├── strawberry_bt_plugins/
│   ├── strawberry_moveit_plugins/
│   └── strawberry_visp_utils/
├── apps/                     # 실행 노드와 런치
│   ├── task_planner_node/
│   ├── servo_node_launch/
│   └── bringup_launch/
├── sim/                      # Gazebo/Webots 등 선택
│   └── strawberry_gazebo/
├── conf/                     # Hydra 설정 트리 (§ 3)
│   ├── default.yaml
│   ├── env/
│   ├── robot/
│   ├── task/
│   └── servo/
└── .github/
    └── workflows/ci.yaml
```

* 한 레포에 전체 패키지를 두면 패키지 버전·의존성 동기화와 CI 설정이 단순해집니다 ([behaviortree.dev][1]).
* `core/` 계층은 ROS 2 헤더를 포함하지 않아 다른 프로젝트로 그대로 복사할 수 있습니다.

---

## 2. 패키지별 세부 역할

| 패키지                             | 노드/라이브러리           | 핵심 기능                                                                                               |
| ------------------------------- | ------------------ | --------------------------------------------------------------------------------------------------- |
| **strawberry\_core**            | 0 노드               | 기하·필터·보정 수학 모듈, `Eigen` · `yaml-cpp`                                                                |
| **strawberry\_description**     | URDF · meshes      | `tool0`, `camera_optical`, `basket_link` 등 정합                                                       |
| **strawberry\_moveit\_config**  | MoveIt SRDF + yaml | Servo/Hybrid Planning 파이프라인 기본값 ([moveit.picknik.ai][2], [moveit.picknik.ai][3])                    |
| **strawberry\_bt\_plugins**     | BT 플러그인 SO         | `MoveToPredefined`, `PlanToPose`, `VisualServo` 등 ROS2 Action/Service 래퍼 ([GitHub][4], [GitHub][5]) |
| **strawberry\_moveit\_plugins** | MoveIt plugin SO   | 커스텀 LocalPlanner · TrajectoryOperator for Hybrid Planning ([moveit.picknik.ai][3])                  |
| **strawberry\_visp\_utils**     | lib + node         | `visp_bridge` 기반 EEF Twist 계산 · publish ([GitHub][6])                                               |
| **task\_planner\_node**         | 실행 노드              | `behaviortree_ros2::BTExecutor` 구동, XML 핫스왑 지원 ([behaviortree.dev][1])                              |
| **servo\_node\_launch**         | 런치                 | MoveIt Servo 단독/컴포넌트 모드, 파라미터 주입 ([moveit.picknik.ai][2])                                           |
| **bringup\_launch**             | 런치                 | URDF → controllers → MoveGroup → Servo → Planner 한 번에                                               |

> **플러그인 ABI 주의**: BT·MoveIt 플러그인은 모두 **C++20 + RelWithDebInfo** 동일 컴파일 옵션이어야 런타임 로딩 충돌을 피합니다.

---

## 3. Hydra 구성 – 모든 실험 파라미터의 단일 진입점

```
conf/
├── default.yaml            # 공통: robot=strawberry_v1, env=sim, task=harvest
├── env/
│   ├── sim.yaml            # Gazebo 주행 QoS, fake_controllers
│   └── real.yaml           # 실제 EtherCAT driver IP
├── robot/
│   ├── strawberry_v1.yaml  # URDF 경로, link 길이
│   └── strawberry_v2.yaml
├── task/
│   ├── harvest.yaml        # BT 트리 XML, servo gains
│   └── deposit.yaml
└── servo/
    └── panda_default.yaml  # MoveIt Servo 40+ 매개변수 기본값
```

### 3-1 런치 파일에서 Hydra → ROS 2 파라미터로 주입

```python
from hydra import compose, initialize
from launch_ros.actions import Node

def generate_launch_description():
    initialize(config_path=os.environ["HYDRA_CONFIG_DIR"])
    cfg = compose(config_name="default")
    servo_cfg = cfg["servo"].to_container(resolve=True)

    return LaunchDescription([
        Node(
            package="moveit_servo",
            executable="servo_node",
            parameters=[
                PathJoinSubstitution([FindPackageShare("strawberry_moveit_config"), "config/servo_parameters.yaml"]),
                servo_cfg                                   # Hydra dict
            ]
        )
    ])
```

* MoveIt Servo는 `linear_scale`, `rotational_scale` 등 40여 개 파라미터를 런타임에 변경하면 떨림을 크게 줄일 수 있습니다 ([moveit.picknik.ai][2]).
* Hydra `--multirun` 옵션으로 `servo.gain=0.5,0.7,1.0` 같은 파라미터 스윕을 즉시 실행할 수 있습니다 ([Hydra][7]).

---

## 4. Behavior Tree & MoveIt 파이프라인 연결

```xml
<root main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Fallback name="PoseSelector">
      <Sequence>
        <IsCmd   pose="rest"/>
        <MoveToPredefined pose="rest"/>
      </Sequence>
      <Sequence>
        <IsCmd   pose="deposit"/>
        <MoveToPredefined pose="deposit"/>
      </Sequence>
      <Sequence name="Harvest">
        <IsCmd   pose="harvest"/>
        <MoveToPredefined pose="ready"/>
        <PlanToPose target="rough_target" group="arm"/>
        <ExecutePlan  group="arm"/>
        <VisualServo  topic="eef_twist"/>
        <MoveToPredefined pose="deposit"/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
```

* `MoveToPredefined`, `PlanToPose` 같은 노드는 **behavior\_tree\_moveit2\_nodes** 예제를 그대로 사용해 바로 빌드할 수 있습니다 ([GitHub][5]).
* VisualServo 노드는 ViSP가 발행한 `TwistStamped`를 MoveIt Servo 주제(`/servo_node/cartesian_twist_cmds`)로 중계만 하면 됩니다 ([GitHub][6], [moveit.picknik.ai][2]).
* Groot2를 켜면 트리 tick · 블랙보드를 실시간으로 모니터링할 수 있어 디버그 효율이 매우 높습니다 ([Robotics Stack Exchange][8]).

---

## 5. MoveIt 2 설정 포인트

| 기능                  | 설정 파일 / 플러그인                                                                     | 참고                                                                         |
| ------------------- | -------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| **사전 자세(1,2,3)**    | `robot/config/rest.yaml` 등 → `strawberry_moveit_config` 안 `joint_limits.yaml` 유지 | MoveGroup API의 `setJointValueTarget()`이 일관된 결과 보장 ([moveit.picknik.ai][2]) |
| **Servo**           | `servo_parameters.yaml` + 런치 주입                                                  | 싱귤래리티 · 충돌 완화 내장 ([moveit.picknik.ai][2])                                  |
| **Hybrid Planning** | `GlobalPlanner`, `LocalPlanner`, `PlannerLogic` 플러그인 세트                          | 온라인 재계획·반응형 이벤트 처리 ([moveit.picknik.ai][3])                                |

---

## 6. CI / CD — GitHub Actions 두 단계

```yaml
# .github/workflows/ci.yaml
name: CI
on: [push, pull_request]

jobs:
  ros_ci:
    uses: ros-tooling/action-ros-ci@v0
    with:
      target-ros2-distro: humble
      package-name: |
        strawberry_core
        strawberry_bt_plugins
        task_planner_node

  industrial_ci:
    uses: ros-industrial/industrial_ci@master
    with:
      ROS_DISTRO: humble
      PACKAGE_NAME: ''
```

* `action-ros-ci`는 colcon build + test를 원스텝으로 제공하여 빠른 피드백을 줍니다 ([GitHub][9]).
* MoveIt 공식도 `industrial_ci`를 병행 사용해 빌드팜과 동일 조건의 테스트를 돌립니다 ([MoveIt][10]).

---

## 7. Launch 패턴 & rclcpp Components

* `bringup_launch.py` → URDF · controllers · MoveGroup · Servo · Hybrid Planning을 **ComposableNodeContainer**에 로드해 intra-process QoS 최적화 ([ROS][11]).
* `task_planner.launch.py` 에서는 `BTExecutor`만 별도 프로세스로 두면, 플러그인 reload 시 다른 노드가 죽지 않습니다 ([behaviortree.dev][1]).
* 대규모 프로젝트 런치 파일 분할·재사용 패턴은 ROS 공식 “Managing large projects” 가이드의 `IncludeLaunchDescription` 구조를 그대로 따릅니다 ([ROS][12]).

---

## 8. 실로봇 적용 체크리스트

1. **TF 고정** : `base_link → camera_optical_frame → tool0` 체인은 static TF로 한 번만 선언해야 Servo Twist 오차가 0입니다 ([moveit.picknik.ai][2]).
2. **Servo 파라미터 튜닝** : `low_pass_filter_coeff`, `joint_stopped_velocity_threshold` 값을 Hydra로 실험하세요. MoveIt Servo 튜토리얼의 기본값을 기준 삼으면 안전합니다 ([moveit.picknik.ai][2]).
3. **Hybrid Planning 이벤트 처리** : 딸기 줄기가 갑자기 움직일 때 LocalPlanner가 충돌 예측 이벤트를 보내면 PlannerLogic Plugin이 즉시 재계획을 트리거하게 만듭니다 ([moveit.picknik.ai][3]).
4. **BT 예외 복구** : `RetryUntilSuccessful` 데코레이터와 Servo status topic을 묶어 실패 시 N 회 재시도 후 `MoveToPredefined(rest)` 로 안전 복귀.

---

## 9. 다음 단계 제안

* MoveIt Pro (OSS 아님)의 Behavior Suite를 도입하면 170+ 사전 제작 노드로 BT 구성이 더욱 단순해집니다 ([Picknik Docs][13]).
* Semantic Scene Graph + TF2 Dynamic Broadcaster로 바구니가 이동할 때도 Pose 2를 자동 보정하도록 확장 가능합니다.
* Gazebo → 실제 HW 전환 시 `env/real.yaml` 에 EtherCAT 컨트롤러 IP·QoS만 바꾸면 나머지 코드는 그대로입니다.

---

이 구조를 그대로 복제한 뒤 **Hydra CLI** 로 파라미터를 덮어쓰며 실험해 보세요. *예)*

```bash
# 시뮬레이터에서 Servo gain, 비전 서보 주기만 바꿔 12가지 조합 자동 실험
python -m strawberry_ws.apps.task_planner_node -m env=sim \
    servo.gain=0.5,0.7,1.0 visp.rate=60,120,240
```

설계 단계에서 모듈·플러그인·파라미터를 철저히 분리해 두면, 딸기에서 포도·토마토로 품목이 바뀌어도 **URDF·Hydra yaml 두 파일만 교체**하면 끝! 🚀

[1]: https://www.behaviortree.dev/docs/ros2_integration/ "Integration with ROS2 | BehaviorTree.CPP"
[2]: https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html "Realtime Arm Servoing — MoveIt Documentation: Humble  documentation"
[3]: https://moveit.picknik.ai/humble/doc/examples/hybrid_planning/hybrid_planning_tutorial.html "Hybrid Planning — MoveIt Documentation: Humble  documentation"
[4]: https://github.com/BehaviorTree/BehaviorTree.ROS2 "GitHub - BehaviorTree/BehaviorTree.ROS2: BehaviorTree.CPP utilities to work with ROS2"
[5]: https://github.com/wienans/behavior_tree_moveit2_nodes?utm_source=chatgpt.com "Moveit2 Nodes for BehaviorTree.CPP - GitHub"
[6]: https://github.com/lagadic/vision_visp?utm_source=chatgpt.com "lagadic/vision_visp: ViSP stack for ROS - GitHub"
[7]: https://hydra.cc/docs/1.0/tutorials/basic/running_your_app/multi-run/?utm_source=chatgpt.com "Multi-run | Hydra"
[8]: https://robotics.stackexchange.com/questions/109977/is-groot-still-used-to-monitor-nav2-behaviour-tree-in-2024-ros2-humble?utm_source=chatgpt.com "Is Groot still used to monitor nav2 behaviour tree in 2024? (ROS2 ..."
[9]: https://github.com/ros-tooling/action-ros-ci "GitHub - ros-tooling/action-ros-ci: Github Action to build and test ROS 2 packages using colcon"
[10]: https://moveit.ai/documentation/contributing/continuous_integration/?utm_source=chatgpt.com "MoveIt's Continuous Integration"
[11]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-a-Composable-Node.html?utm_source=chatgpt.com "Writing a Composable Node (C++) — ROS 2 Documentation"
[12]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html "Managing large projects — ROS 2 Documentation: Humble  documentation"
[13]: https://docs.picknik.ai/concepts/behavior_trees/?utm_source=chatgpt.com "Behavior Trees | MoveIt Pro Docs"

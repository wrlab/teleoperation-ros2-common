# Teleoperation ROS 2 공통 라이브러리

## 프로젝트 목적
이 저장소는 WXR 원격 조작(teleoperation) ROS 2 스택에서 반복적으로 사용하는 C++ 및 Python 유틸리티를 모아 제공합니다. 활동 추적, 구조화된 로깅, QoS 도우미처럼 여러 로봇 애플리케이션에서 공통으로 필요한 기능을 묶어두어, 각 애플리케이션은 임무별 로직에 집중하면서도 일관된 계측 환경을 유지할 수 있습니다.

## 저장소 구성
- `src/wxr_ros2_common_cpp`: 모든 `rclcpp` 노드에서 재사용할 수 있는 헤더 전용 C++ 도우미 모음입니다.
- `src/wxr_ros2_common_py`: `rclpy` 노드에서 동일한 개념을 활용하도록 하는 ament Python 패키지입니다.

두 패키지는 독립적으로 빌드·배포할 수 있지만, 하나의 ROS 2 워크스페이스 안에서 함께 사용하는 것을 권장합니다.

## 필요 ROS 2 의존성
다음 도구가 갖춰진 ROS 2 환경을 전제로 합니다.
- C++ 헬퍼를 위한 `ament_cmake` 빌드 도구와 `rclcpp` 헤더
- Python 헬퍼를 위한 `ament_python` 빌드 타입과 `rclpy` 호환 QoS/프로파일 객체
- 품질 검사를 실행하기 위한 `ament_lint_auto`, `ament_lint_common`, `ament_flake8`, `ament_pep257`, `python3-pytest`

## 빌드 및 설치 방법
1. 사용하려는 ROS 2 배포판을 먼저 소싱합니다.
   ```bash
   source /opt/ros/$ROS_DISTRO/setup.bash
   ```
2. 저장소 루트에서 두 패키지를 `colcon`으로 빌드합니다.
   ```bash
   colcon build --packages-select wxr_ros2_common_cpp wxr_ros2_common_py
   ```
3. 노드를 실행하기 전에 오버레이를 소싱합니다.
   ```bash
   source install/setup.bash
   ```

이 과정을 거치면 C++ 헤더는 워크스페이스의 include 경로에 설치되고, Python 유틸리티는 소싱된 환경에서 import 가능해집니다.

## 린트 및 테스트 실행
각 패키지에 포함된 린트 구성을 `colcon test`로 실행할 수 있습니다.
```bash
colcon test --packages-select wxr_ros2_common_cpp
colcon test --packages-select wxr_ros2_common_py
colcon test-result --verbose
```
Python 패키지는 저작권 검사, Flake8, PEP 257을 포함하고, C++ 패키지는 통합된 ament lint auto 프로파일을 사용합니다.

## 사용 예시
### C++ 활동 추적 및 구조화된 로깅
```cpp
#include "rclcpp/rclcpp.hpp"
#include "wxr_ros2_common_cpp/activity_tracker.hpp"
#include "wxr_ros2_common_cpp/structured_logging.hpp"
#include "wxr_ros2_common_cpp/qos_utils.hpp"

class TeleopNode : public rclcpp::Node {
public:
  TeleopNode() : Node("teleop_node"), tracker_(*this) {
    tracker_.markSub("/joy");
    tracker_.markPub("/cmd_vel");

    auto qos = rclcpp::QoS{10};
    wxr_ros2::logEndpoint(*this, "[Sub]", "/joy", "sensor_msgs/msg/Joy", wxr_ros2::qosToBriefString(qos));
  }

private:
  wxr_ros2::ActivityTracker tracker_;
};
```
`ActivityTracker`는 주기적으로 어떤 토픽이 사용 중인지 출력하고, `logEndpoint`와 `qosToBriefString`의 조합은 일관된 엔드포인트 메타데이터 로그를 남깁니다.

### Python 유틸리티
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from wxr_ros2_common_py.activity_tracker import ActivityTracker
from wxr_ros2_common_py.structured_logging import log_startup, log_latency
from wxr_ros2_common_py.qos_utils import qos_to_multiline

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self._tracker = ActivityTracker(self)
        self._tracker.mark_sub('/joy')
        self._tracker.mark_pub('/cmd_vel')

        qos = QoSProfile(depth=10)
        log_startup(
            self,
            params={'mode': 'manual'},
            endpoints=[{'dir': 'sub', 'topic': '/joy', 'type': 'sensor_msgs/msg/Joy', 'qos': qos}],
        )

        start = self.get_clock().now().nanoseconds / 1e9
        # ... 작업 수행 ...
        end = self.get_clock().now().nanoseconds / 1e9
        log_latency(self, 'loop', start, end)
```
`ActivityTracker`는 C++ 도우미와 동일하게 동작하며, 구조화된 로깅 유틸리티는 파라미터·엔드포인트·지연 시간 정보를 QoS 포맷과 함께 정리된 로그로 제공합니다.

# ROS2 기반 벽 따라가기 및 Stop sign 인식 후 정지 프로젝트

> 본 프로젝트는 **2025년 자율주행로봇 전문가 과정(광운대학교 공학교육혁신센터)** 의 일환으로 진행되었다. 총 40시간의 집중 교육 과정 중 마지막 2일 동안 구현되었다. ROS2 환경에서 LiDAR 데이터를 이용한 벽 따라가기와 YOLOv5 기반의 Stop Sign 표지판 인식을 결합한 통합 제어 시스템이다. 실제 로봇의 하드웨어 제약(GPU 부재)을 극복하기 위해 ONNX Runtime을 활용한 CPU 최적화 추론을 구현했으며 Docker 기반의 시뮬레이션 환경과 실제 로봇 플랫폼(Storagy)에서 동작을 확인했다.

<br>

## Demo Video
실제 로봇(Storagy)에서 벽 따라가기 및 Stop Sign 인식 후 정지 동작을 수행하는 데모 영상

https://youtu.be/UZNEorQzys4

<br>

## Tech Stack
- Framework: ROS2 (Humble)
- Language: Python 3.11
- AI Framework: YOLOv5 (v7.0), ONNX Runtime
- Infrastructure: Docker, Ubuntu 22.04
- Sensors: LiDAR (SICK 500 series), RGB Camera

<br>

## AI Model & Training
본 프로젝트는 제한된 개발 기간과 하드웨어 환경에서 ROS2 기반 주행 제어 시스템 구현을 목표로 진행되었으며 모델 성능의 극대화보다는 실시간 추론 안정성을 우선하였다.

### 1. 모델 사양
- Architecture: YOLOv5n (Nano)
- Optimization: Exported to ONNX for CPU-only inference
- Parameters: 약 176만 개 (1,760,518)
- Input Size: 640 x 640

### 2. Dataset 정보
- Source: Roboflow
- Project: h2politostop
- Version: v6
- Classes: Stop Sign (single class)
- Environment: Indoor track

※ 데이터셋은 Roboflow 플랫폼을 통해 관리되었으며 API Key 및 다운로드 스크립트는 보안상 공개하지 않음.

### 3. Training 성능 (Test Set)
- Precision: 0.999
- Recall: 1.000 (100%)
- mAP50: 0.995
- Inference Speed: i3 CPU 환경에서 약 20 FPS(50ms) 확보 (실제 Storagy는 i5 cpu 환경)

※ 상기 성능 지표는 단일 클래스 데이터셋을 기준으로 측정된 결과임. ONNX 모델 파일은 용량 및 라이선스 이슈로 저장소에 포함하지 않음.

<br>

## Key Features
본 프로젝트에서 AI 모델은 주행 제어 이벤트 트리거를 위한 보조 모듈로 활용되었다.

### 1. Hybrid Development Environment
- Dockerized Workflow: Docker 및 Dockerfile을 활용하여 환경 의존성을 제거하고 시뮬레이션 환경(Gazebo/RViz2)을 신속하게 구축

- Real-robot Integration: SSH를 통한 실제 물류 로봇 'Storagy' 접속 및 네트워크 설정을 통해 하드웨어 배포 프로세스를 수행

### 2. Control Logic
- LiDAR 센서를 이용해 벽 따라가기 알고리즘을 구현
- YOLOv5n 모델이 Stop Sign 감지 시 ROS2 토픽 기반 3초 정지 이벤트 수행

<br>

## Troubleshooting
### 1. 특정 하드웨어 종속성 및 라이브러리 충돌
- 문제: 실제 로봇(Storagy)과 개인 PC의 라이브러리 버전 불일치로 인한 실행 불가 현상.
- 해결: Docker 컨테이너 기술을 도입하여 독립된 개발 환경을 구축하고 소프트웨어 이식성을 확보.

### 2. SLAM 매핑 중 지도 흔들림 및 왜곡
- 문제: 로봇의 급격한 회전 시 LiDAR 데이터 정합성이 떨어져 지도가 왜곡됨.

- 해결: 키보드 제어 시 선속도와 각속도를 낮게 고정하여 부드러운 주행을 유도하고 반복 주행을 통해 데이터 신뢰도를 높임.

### 3. LiDAR 센서 사양 불일치 해결
- 문제: 시뮬레이션(360°)과 실제 SICK LiDAR(약 160°)의 유효 화각 차이로 인한 데이터 슬라이싱 인덱스 오류 발생.

- 해결: 입력 데이터 범위를 160°로 재설정하고 각도 기반 동적 인덱스 계산 함수(_region_median)를 구현하여 소프트웨어 유연성을 확보.

<br>

## 개선 방향
### 1. 제어 알고리즘 고도화
- **곡선 구간 최적화:** 급격한 곡선 구간에서 안정적인 주행을 위해 선속도 감속 및 조향각을 정교하게 계산하는 적응형 제어 로직 추가.

### 2. AI 모델 성능 및 기능 확장
- **멀티 클래스 탐지:** Stop Sign 외에도 신호등, 동적 장애물(사람) 등을 추가 학습하여 복잡한 환경 대응력 강화.
- **환경 강건성 확보:** 실내 트랙을 넘어 가변적인 조도와 배경을 가진 실외 환경에서도 안정적으로 동작하는 모델 학습.

### 3. 시스템 안정성 강화
- **비상 정지 로직:** 통신 단절, 센서 오류 또는 예상치 못한 시스템 중단 발생 시 로봇을 즉각 안전하게 정지시키는 하드웨어-소프트웨어 통합 비상 정지 시스템 강화.

### 4. 중복 정지 방지 로직 구현
- **상태 플래그:** 현재 로봇이 '정지 프로세스' 중인지 확인하여 이미 멈춰있는 동안 들어오는 추가 신호는 모두 무시.

- **쿨타임 타이머:** 정지가 해제된 직후, 로봇이 해당 표지판을 완전히 통과할 수 있도록 일정 시간(예: 5~10초, 현재는 3초) 동안은 정지 인식을 강제로 비활성화.

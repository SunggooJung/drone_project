---
name: BASALT 커스텀 VIO 전략 + 로드맵
description: 고속드론 VIO 비교, BASALT 개조 전략, 속도별 목표 및 현실적 개발 일정 (검증 상태 명시)
type: project
---

## 범례
- ✅ 공식 스펙/코드/논문 확인
- ⚠️ 추정 또는 미검증
- ❌ 오류 수정됨
- 🔲 실측 필요

---

## 목표 속도

- **MVP**: 5-10 m/s 안정적 실외 VIO 시연
- **최종**: 10-20 m/s

---

## BASALT vs cuVSLAM 핵심 비교

| 항목 | BASALT | cuVSLAM | 검증 |
|------|--------|---------|------|
| 알고리즘 | Patch optical flow + Sliding window BA | FAST descriptor + GPU BA | ✅ |
| Loop closure | 없음 (placeholder만 존재) | 내장 | ✅ 코드 확인 |
| 고속 motion blur | **강함** | 약함 | ✅ 알고리즘 특성 |
| 장거리 drift | 누적 위험 | loop closure로 보정 | ✅ |
| Orin NX 최적화 | 없음 | Isaac ROS 최적화 | ✅ |
| IMU 강인성 | **Preintegration 정교함** | 표준 수준 | ✅ |

**EuRoC 성능 (확인된 수치):**
- Basalt ATE: ~0.012m (MH sequences) ✅ 논문 기반
- OpenVINS RMSE: ~0.021m (V1_01) ✅ 공식 문서 기반
- Orin NX에서의 실제 latency: 🔲 직접 측정 필요

---

## BASALT 아키텍처 (코드 직접 분석, 검증됨)

| 항목 | 상태 | 검증 |
|------|------|------|
| IMU-aided feature prediction | **없음** | ✅ 코드 확인 |
| Schur complement marginalization | 구현됨 | ✅ 코드 확인 |
| IMU preintegration | 구현됨 | ✅ 코드 확인 |
| IMU-OpticalFlow 분리 구조 | 분리됨 (back-end에서만 IMU 사용) | ✅ 코드 확인 |
| Loop closure | Placeholder만 있음 | ✅ 코드 확인 |
| 수정 진입점 | OpticalFlowInput, trackPointAtLevel() | ✅ 코드 확인 |

---

## 속도별 안정성 분석

```
< 5 m/s     현재 계획으로 안정적       ✅ 알고리즘 근거
5-10 m/s    진동/캘리브레이션 관리 시 가능 (MVP 목표) ⚠️ 실측 전
10-15 m/s   SuperPoint fallback + 온라인 재보정 필요  ⚠️ 실측 전
15 m/s+     이벤트 카메라 없으면 매우 어려움         ✅ 알고리즘 한계
```

---

## 하드웨어 위험 요소 (실측 전 해결 불가)

1. **진동**: 레이싱 드론 실측 IMU noise = 스펙의 10-70배 ✅ 알려진 문제
2. **캘리브레이션 안정성**: 충격/열팽창 extrinsic drift ✅
3. **하드웨어 sync**: D455 내장 sync ✅, OV9281+Pixhawk 수동 구현 필요 ✅
4. **IMU 포화**: ICM-42688-P 최대 ±2000 dps ✅ 데이터시트 확인
5. **VIO 전용 IMU**: FC IMU(soft mount)와 hard-mount IMU 분리 필수 ✅

---

## 개조 로드맵 (현실성 검토 완료)

### Phase 1 — MVP 필수 (Month 1-3)

**1-1. IMU-aided Feature Prediction** ← 가장 임팩트 ✅
- 임팩트: 고속 tracking 실패의 70% 해결 ⚠️ 추정, 실측 필요
- 핵심: OpticalFlowInput에 predicted_T_prev_curr 추가
- trackPointAtLevel() SE(2) 초기값을 IMU prediction으로 교체
- 도전: Depth ambiguity → rotation-only prediction으로 우선 해결
- 수정 파일: `optical_flow.h`, `frame_to_frame_optical_flow.h`, `vio.cpp` ✅ 확인
- 예상 기간: 4-5주 (통합/디버깅 포함) ⚠️ 추정

**1-2. 하드웨어 연동**
- D455 realsense-ros2 + Basalt 연동
- Kalibr 캘리브레이션 (factory calibration보다 custom 권장)
- VIO 전용 IMU hard-mount 구성

### Phase 2 — 고속 확장 (Month 4-5)

**2-1. OV9281 + Pixhawk IMU 전환**
- ICM-42688-P: 2.8 mdps/√Hz ✅ (BMI055 대비 개선)
- 하드웨어 트리거 sync 구현 (Pixhawk AUX6 → OV9281)
- 주의: OV9281 Jetson 실제 FPS = 70fps ✅ (120fps 아님)

**2-2. SuperPoint Fallback (GPU)**
- 트리거: tracked_ratio < 0.5 → SuperPoint TensorRT
- Orin NX GPU 추론 시간: 🔲 실측 필요 (이전 ~8ms 추정은 미검증)
- 예상 기간: 3-4주 ⚠️

**2-3. 온라인 Extrinsic 재보정**
- 비행 중 camera-IMU extrinsic 미세 보정 (VINS-Mono 방식)

**2-4. 실패 감지 + 재초기화**
- covariance 폭증 감지 → 자동 재초기화

### Phase 3 — 장거리 대응 (Month 6+)

**3-1. DBoW3 Loop Closure**
- 실제 소요: 8-12주 ✅ (3-4주는 낙관적)
- 핵심 난이도: marginalize된 프레임에 loop closure 적용

---

## 제외 항목 (이유 포함)

**❌ CUDA Optical Flow 교체**
- VIO feature count(150-300개)에서 GPU kernel overhead로 CPU TBB와 성능 유사
- Basalt SE(2) patch tracker가 OpenCV CUDA LK보다 알고리즘 품질 우수
- ROI 낮음 ✅ 검토 완료

**❌ GPU Bundle Adjustment**
- Eigen sparse solver → CUDA 재작성 수준
- Orin NX CPU BA로 실시간 충분 ⚠️ 실측 전 가정

---

## MVP 성공 기준

```
위치 drift:    < 1% of distance traveled (100m → < 1m)
VIO latency:   목표 < 15ms (🔲 Orin NX 실측 후 조정)
Tracking 유지: > 80% of flight time
속도:          5-10 m/s 안정적
```

---

## 전체 개발 일정

```
Month 1: Basalt 빌드 + D455 EuRoC 베이스라인 + 하드웨어 조립
Month 2: IMU-aided prediction + Kalibr 캘리브레이션 + 실하드웨어 연동
Month 3: 실외 5→10 m/s 테스트 + PX4 MAVLink 연동  ← MVP 완성
Month 4: OV9281 전환 + SuperPoint fallback
Month 5: 10-20 m/s 고속 테스트 + 튜닝
Month 6+: Loop closure
```

---

## 다음 즉시 할 일

1. Basalt Orin NX 빌드 스크립트 작성
2. D455 + realsense-ros2 + Basalt 연동 설정
3. EuRoC 돌려서 latency 실측 (모든 계획의 기준점)

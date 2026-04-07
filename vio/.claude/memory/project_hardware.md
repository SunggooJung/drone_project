---
name: Hardware Configuration
description: 350급 고속 자율비행 VIO 드론 하드웨어 스펙 (검증 상태 명시)
type: project
---

350mm급 고속 자율비행 VIO 드론 MVP 하드웨어 구성.

**Why:** Orin NX 16GB 탑재 + 고속 실외 VIO 목표 → 300급은 추력비 부족, 350급으로 결정
**How to apply:** 부품 추천, 소프트웨어 설계, 캘리브레이션 절차 등 이 스펙 기준으로

---

## 범례
- ✅ 공식 스펙시트/제품 페이지 확인
- ⚠️ 추정 또는 비공식 출처
- ❌ 오류 수정됨
- 🔲 구매/실측 전 확인 필요

---

## 기체

| 항목 | 선택 | 무게 | 검증 |
|------|------|------|------|
| 프레임 | iFlight Chimera7 Pro V2 또는 Diatone Roma F5 V2 | ~100g | ⚠️ 추정 |
| 모터 | T-Motor F40 Pro IV 2306 2450KV x4 | 29.5g/ea (118g 합계) | ✅ T-Motor 공식 |
| ESC | Holybro Kotleta20 4in1 또는 T-Motor F45A 4in1 | ~30g | ⚠️ 추정 |
| 배터리 | 4S 2200mAh 100C LiPo | ~180g | ⚠️ 추정 |
| 프롭 | 5인치 | ~15g | ⚠️ 추정 |

**모터 추력 (검증됨):**
- T-Motor F40 Pro IV 2306 2450KV, 4S, 5.1×5 prop: **1902g/motor** ✅
- 전체 추력: 7608g
- ⚠️ 최종 추력비는 실제 총 무게 측정 후 재계산 필요

---

## 비행 제어

| 항목 | 선택 | 검증 |
|------|------|------|
| FC | Holybro Pixhawk 6C Mini | ✅ Holybro 공식 |
| 무게 | **42.4g** (Model A 기준) | ✅ Holybro 공식 |
| IMU | ICM-42688-P + BMI088 (듀얼 IMU) | ✅ Holybro 공식 |
| Camera Trigger | AUX 핀 (AUX6, GPIO55) 사용 가능 | ✅ ArduPilot 문서 |
| 펌웨어 | PX4 | ✅ 공식 지원 |

**IMU 스펙 비교:**
- ICM-42688-P gyro noise: **2.8 mdps/√Hz** ✅ TDK 데이터시트
- BMI055 gyro noise: ⚠️ 데이터시트 직접 확인 불가 (일반적으로 ~10-14 mdps/√Hz로 알려짐, 미검증)
- BMI088 gyro noise: ~6 mdps/√Hz ⚠️ 추정 (Pixhawk 6C Mini의 보조 IMU)

---

## 컴퓨트

| 항목 | 선택 | 검증 |
|------|------|------|
| SBC | Jetson Orin NX 16GB | ✅ NVIDIA 공식 |
| CPU | 8-core ARM Cortex-A78AE | ✅ NVIDIA 공식 |
| GPU | 1792-core Ampere, 56 Tensor Core | ✅ NVIDIA 공식 |
| MIPI CSI | 8 lanes (최대 4 카메라) | ✅ NVIDIA 공식 |
| 모듈 무게 | ⚠️ 공식 미공개 (~30-60g 추정, 미검증) | 🔲 직접 계량 필요 |
| 캐리어보드 | **ConnectTech Hadron NGX012** | ✅ ConnectTech 공식 |
| 캐리어 크기 | 82.6 × 58.8mm | ✅ ConnectTech 공식 |
| 캐리어 무게 | ⚠️ 공식 미공개 | 🔲 직접 계량 필요 |
| 캐리어 MIPI CSI | 1× 4-lane (스테레오 구성 시 별도 검토 필요) | ✅ ConnectTech 공식 |

> ⚠️ **중요**: 이전에 언급한 ConnectTech Quark는 Orin NX 미지원 (Xavier NX/Nano용).
> Orin NX 올바른 제품: **ConnectTech Hadron NGX012**

---

## MVP 카메라 — D455

| 항목 | 스펙 | 검증 |
|------|------|------|
| 모델 | Intel RealSense D455 | ✅ |
| IR 스테레오 셔터 | **Global Shutter** | ✅ Intel 공식 |
| RGB 셔터 | **Global Shutter** | ✅ Intel 공식 |
| 최대 FPS | 90fps (IR + RGB 모두) | ✅ Intel 공식 |
| 베이스라인 | 95mm | ✅ Intel 공식 |
| IMU | BMI055 | ✅ Intel 공식 |
| 인터페이스 | USB 3.1 Type-C | ✅ Intel 공식 |
| Camera-IMU Sync | 하드웨어 동기화 (동일 ASIC 클럭) | ✅ Intel 공식 |

---

## Phase 2 카메라 — OV9281 스테레오

| 항목 | 스펙 | 검증 |
|------|------|------|
| 모델 | Arducam OV9281 스테레오 키트 | ✅ Arducam 공식 |
| 셔터 | Global Shutter | ✅ |
| 해상도 | 1280×800 | ✅ |
| Jetson 실제 FPS | **70fps** (센서 최대 120fps이나 Jetson에서 70fps) | ✅ Arducam 공식 |
| 색상 | Monochrome | ✅ |
| 인터페이스 | MIPI CSI-2 | ✅ |
| 무게 | ⚠️ 공식 미공개 | 🔲 구매 후 계량 필요 |

> ⚠️ Hadron NGX012의 MIPI CSI가 1x 4-lane이므로
> OV9281 스테레오 연결 시 Arducam Camarray HAT 또는 다른 방법 검토 필요

---

## 마운팅 원칙

- 카메라 + VIO IMU → 3mm 카본 플레이트에 강체 마운트 → 프레임 직결 (진동 흡수 마운트 금지)
- FC IMU (soft mount) ≠ VIO IMU (hard mount) — 반드시 분리
- 카메라 위치: 프레임 전방, 전방 15도 하향각
- Pixhawk AUX6 (GPIO55) → OV9281 하드웨어 트리거

---

## 무게 예산 (검증 상태 포함)

| 항목 | 무게 | 검증 |
|------|------|------|
| 프레임 | ~100g | ⚠️ |
| 모터 x4 | **118g** | ✅ |
| ESC | ~30g | ⚠️ |
| 프롭 | ~15g | ⚠️ |
| Pixhawk 6C Mini | **42.4g** | ✅ |
| Orin NX 모듈 | 미공개 | 🔲 |
| Hadron 캐리어 | 미공개 | 🔲 |
| D455 (MVP) | ~120g | ⚠️ |
| 배선/마운트 | ~40g | ⚠️ |
| 배터리 | ~180g | ⚠️ |
| **합계** | **🔲 실제 조립 후 계량 필요** | |

> ⚠️ 이전 648g 추정은 미검증 수치 기반. 실제 조립 후 재측정 필수.

---

## 소프트웨어 스택

| 항목 | 선택 | 검증 |
|------|------|------|
| OS | Ubuntu 22.04 + JetPack 6.x | ✅ NVIDIA 공식 지원 |
| 미들웨어 | ROS2 Humble | ✅ |
| VIO (MVP) | Basalt | ✅ 코드 분석 완료 |
| VIO (Phase 2) | Basalt 커스텀 | ✅ 아키텍처 설계 완료 |
| 캘리브레이션 | Kalibr | ✅ |
| MVP 카메라 드라이버 | realsense-ros2 | ✅ 공식 지원 |

---

## 성능 수치 (미측정)

아래 수치는 모두 **Orin NX에서 실측 전까지 추정값**:

| 항목 | 추정 | 상태 |
|------|------|------|
| Basalt latency | 미측정 | 🔲 EuRoC 테스트 후 측정 |
| OpenVINS latency | 미측정 | 🔲 |
| SuperPoint TensorRT | 미측정 | 🔲 |
| 비행시간 | 미측정 | 🔲 조립 후 측정 |

---

## MVP 성공 기준

```
위치 drift:    < 1% of distance (100m → < 1m 오차)
VIO latency:   목표 < 15ms (실측 후 조정)
Tracking 유지: > 80% of flight time
속도:          5-10 m/s 안정적
```

## 개발 순서

```
Phase 1 (MVP): D455 + Basalt + 기본 세팅      → 5-10 m/s
Phase 2:       OV9281 + IMU-aided prediction   → 10-15 m/s
Phase 3:       SuperPoint fallback + 튜닝      → 15-20 m/s
```

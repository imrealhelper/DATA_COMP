# ADMM-Based Trajectory Reconstruction Pipeline

![Figure 1 – Reconstruction Result](Figure_1.png)

> 6-DOF 비행체(미사일·UAV 등) 궤적을 시뮬레이션하고
> 노이즈·결손 구간이 있는 데이터를 **ADMM** 으로 복원합니다.

---

## ✨ 주요 기능

* Monte Carlo 기반 비행 궤적 생성
* 가우시안 노이즈 + 결손 구간 데이터 생성
* **ADMM** 슬라이딩 윈도우 복원
* RMSE / MAE / Max Error 지표 계산 & 시각화

## 🛠️ 요구 사항

* **Python ≥ 3.8**
* 의존성: `requirements.txt` 참조

## ⚡ 설치

```bash
# 1) 가상 환경 생성
python -m venv venv
# Windows
venv\Scripts\activate
# macOS/Linux
source venv/bin/activate

# 2) 패키지 설치
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## 🚀 실행

```bash
python tr.py
```

스크립트가 하는 일

1. 비행 궤적 1개 시뮬레이션
2. 노이즈·결손 데이터 생성
3. ADMM으로 복원
4. 원본·관측·복원 궤적 플롯 출력 → **Figure\_1.png** 저장

## 📊 결과 예시

| 지표       | 설명          |
| -------- | ----------- |
| **RMSE** | 루트 평균 제곱 오차 |
| **MAE**  | 평균 절대 오차    |
| **MAX**  | 최대 절대 오차    |

각 축(X, Y, Z)에 대해 시간이 지남에 따라 변화를 그래프로 확인할 수 있습니다.

## 💡 핵심 최적화 문제

$$
\min_{x}\;\lambda_1 \lVert x - y \rVert_{1} +\; \lambda_2 \lVert x - y \rVert_{2}^{2} +\; \lambda_D \lVert D^{2}x \rVert_{2}^{2}
$$

ADMM으로 **x-update → r-update → dual-update**를 반복하며
슬라이딩 윈도우 방식으로 전체 시계열을 복원합니다.

## 📂 프로젝트 구조

```
ADMM_TRJ/
├── tr.py              # 메인 파이프라인
├── requirements.txt   # 의존 라이브러리
├── Figure_1.png       # 결과 이미지 (자동 생성)
└── README.md          # 바로 이 파일
```

## 👤 Author

**Jinwoo Im** (`imrealhelper`)
Inha University, Dept. of Aerospace Engineering

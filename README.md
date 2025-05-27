# ADMM-Based Trajectory Reconstruction Pipeline

This project simulates a 6-DOF trajectory (e.g., missile or UAV), injects synthetic noise and missing data, and reconstructs the full trajectory using an ADMM-based optimization method in a sliding window fashion.

## 📌 Features

- Monte Carlo simulation of guided ballistic flight
- Corruption with Gaussian noise and missing segments
- ADMM-based denoising and signal reconstruction
- Sliding window recovery with RMSE / MAE / Max Error evaluation
- Visualization of reconstructed and corrupted data

## 🚀 Requirements

- Python 3.8 or higher
- Dependencies listed in `requirements.txt`

## 📦 Installation

Create a virtual environment and install dependencies:

```bash
python -m venv venv
venv\Scripts\activate        # On Windows (cmd)
# OR
source venv/bin/activate     # On macOS/Linux

pip install --upgrade pip
pip install -r requirements.txt
```

## 🏃‍♂️ Run

Execute the main pipeline script:

```bash
python tr.py
```

This will:

1. Generate one simulated launch trajectory.
2. Corrupt the data with noise and missing values.
3. Reconstruct the signal using ADMM.
4. Plot the recovered vs. ground truth vs. corrupted signal.

## 📈 Output

- Sliding window plots with ADMM reconstruction in red
- Quantitative metrics: RMSE / MAE / Max Error over time

## 🧠 Algorithm

The core optimization solves the following problem for each window:

```
minimize   λ₁‖x - y‖₁ + λ₂‖x - y‖₂² + λ_D‖D²x‖₂²
```

Using ADMM to alternate between subproblems, the signal is recovered robustly even in the presence of heavy corruption.

## 📁 Directory Structure

```
ADMM_TRJ/
│
├── tr.py                # Main script
├── requirements.txt     # Dependencies
├── README.md            # You are here
```

## 🧑‍💻 Author

Jinwoo Im (`imrealhelper`)  
Inha University, Dept. of Aerospace Engineering

---

import numpy as np

# Path to save Koopman operators
base = "sa_klqr_controller/config/koopman_ops/"

# --------- Dimensions ---------
n = 6   # 6-state KLQR
m = 1   # scalar control input

# --------- Region 1 ---------
A1 = np.eye(n) + 0.01 * np.random.randn(n, n)
B1 = 0.1 * np.random.randn(n, m)

np.savez(base + "K_1.npz", A=A1, B=B1)
print("Generated K_1.npz")

# --------- Region 2 ---------
A2 = np.eye(n) + 0.01 * np.random.randn(n, n)
B2 = 0.1 * np.random.randn(n, m)

np.savez(base + "K_2.npz", A=A2, B=B2)
print("Generated K_2.npz")

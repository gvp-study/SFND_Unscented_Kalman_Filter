import numpy as np
import matplotlib.pyplot as plt

d = np.loadtxt('nmse-both.txt')
tit = f"RMSE Laser + Radar\nMeans: X {np.mean(d[:,1]):.4f} Y {np.mean(d[:,2]):.4f} Vx {np.mean(d[:,3]):.4f} Vy {np.mean(d[:,4]):.4f}"

plt.figure();
plt.plot(d[:,0], d[:,1], label='X')
plt.plot(d[:,0], d[:,2], label='Y')
plt.plot(d[:,0], d[:,3], label='Vx')
plt.plot(d[:,0], d[:,4], label='Vy')

plt.legend()
plt.title(tit)

plt.show()

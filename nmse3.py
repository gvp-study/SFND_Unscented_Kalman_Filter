import numpy as np
import matplotlib.pyplot as plt

d = np.loadtxt('nmse-both.txt')
dl = np.loadtxt('nmse-laser.txt')
dr = np.loadtxt('nmse-radar.txt')

plt.figure();
plt.plot(d[:,0], d[:,1], label='X')
plt.plot(d[:,0], d[:,2], label='Y')
plt.plot(d[:,0], d[:,3], label='Vx')
plt.plot(d[:,0], d[:,4], label='Vy')
tit1 = f"RMSE Laser+Radar\nMeans: X {np.mean(d[:,1]):.4f} Y {np.mean(d[:,2]):.4f} Vx {np.mean(d[:,3]):.4f} Vy {np.mean(d[:,4]):.4f}"

plt.legend()
plt.title(tit1)

plt.figure();
plt.plot(dl[:,0], dl[:,1], label='X')
plt.plot(dl[:,0], dl[:,2], label='Y')
plt.plot(dl[:,0], dl[:,3], label='Vx')
plt.plot(dl[:,0], dl[:,4], label='Vy')
tit2 = f"RMSE Laser Only\nMeans: X {np.mean(dl[:,1]):.4f} Y {np.mean(dl[:,2]):.4f} Vx {np.mean(dl[:,3]):.4f} Vy {np.mean(dl[:,4]):.4f}"

plt.legend()
plt.title(tit2)

plt.figure();

plt.plot(dr[:,0], dr[:,1], label='X')
plt.plot(dr[:,0], dr[:,2], label='Y')
plt.plot(dr[:,0], dr[:,3], label='Vx')
plt.plot(dr[:,0], dr[:,4], label='Vy')
tit3 = f"RMSE Radar Only\nMeans: X {np.mean(dr[:,1]):.4f} Y {np.mean(dr[:,2]):.4f} Vx {np.mean(dr[:,3]):.4f} Vy {np.mean(dr[:,4]):.4f}"

plt.legend()
plt.title(tit3)

plt.show()

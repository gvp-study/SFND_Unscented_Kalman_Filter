import numpy as np
import matplotlib.pyplot as plt

d = np.loadtxt('nmse-both.txt')
dl = np.loadtxt('nmse-laser.txt')
dr = np.loadtxt('nmse-radar.txt')


fig, axs = plt.subplots(3, 1, sharex=True)

axs[0].plot(d[:,0], d[:,1], label='X')
axs[0].plot(d[:,0], d[:,2], label='Y')
axs[0].plot(d[:,0], d[:,3], label='Vx')
axs[0].plot(d[:,0], d[:,4], label='Vy')
tit1 = f"RMSE Laser+Radar Means: X {np.mean(d[:,1]):.4f} Y {np.mean(d[:,2]):.4f} Vx {np.mean(d[:,3]):.4f} Vy {np.mean(d[:,4]):.4f}"

axs[0].set(ylim=(0, 2))
axs[0].legend()
axs[0].set_title(tit1)

axs[1].plot(dl[:,0], dl[:,1], label='X')
axs[1].plot(dl[:,0], dl[:,2], label='Y')
axs[1].plot(dl[:,0], dl[:,3], label='Vx')
axs[1].plot(dl[:,0], dl[:,4], label='Vy')
tit2 = f"RMSE Laser Only Means: X {np.mean(dl[:,1]):.4f} Y {np.mean(dl[:,2]):.4f} Vx {np.mean(dl[:,3]):.4f} Vy {np.mean(dl[:,4]):.4f}"

axs[1].set(ylim=(0, 2))
axs[1].legend()
axs[1].set_title(tit2)


axs[2].plot(dr[:,0], dr[:,1], label='X')
axs[2].plot(dr[:,0], dr[:,2], label='Y')
axs[2].plot(dr[:,0], dr[:,3], label='Vx')
axs[2].plot(dr[:,0], dr[:,4], label='Vy')
tit3 = f"RMSE Radar Only Means: X {np.mean(dr[:,1]):.4f} Y {np.mean(dr[:,2]):.4f} Vx {np.mean(dr[:,3]):.4f} Vy {np.mean(dr[:,4]):.4f}"

axs[2].set(ylim=(0, 2))
axs[2].legend()
axs[2].set_title(tit3)

plt.show()

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

plt.legend()
plt.title('Laser + Radar')

plt.figure();
plt.plot(dl[:,0], dl[:,1], label='X')
plt.plot(dl[:,0], dl[:,2], label='Y')
plt.plot(dl[:,0], dl[:,3], label='Vx')
plt.plot(dl[:,0], dl[:,4], label='Vy')

plt.legend()
plt.title('Laser only')

plt.figure();

plt.plot(dr[:,0], dr[:,1], label='X')
plt.plot(dr[:,0], dr[:,2], label='Y')
plt.plot(dr[:,0], dr[:,3], label='Vx')
plt.plot(dr[:,0], dr[:,4], label='Vy')

plt.legend()
plt.title('Radar only')

plt.show()

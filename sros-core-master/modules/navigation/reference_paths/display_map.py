#from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

fig = plt.figure()
ax1 = fig.add_subplot(111)
#ax1.set_title('Fig')
plt.xlabel('x(m)')
plt.ylabel('y(m)')

data = numpy.loadtxt('/home/shen/Desktop/map_data.dat')
x = []
y = []
for i in range(len(data)):
    for j in range(len(data[i])):
        if data[i][j] == 0:
            a = 0
        else:
            x.append(j)
            y.append(i)

x.append(0)
y.append(0)
x.append(2000)
y.append(2000)

plt.plot(x, y, '.')

plt.legend(['ref path', 'real trajectory'], loc = 0, ncol = 1)

plt.show()
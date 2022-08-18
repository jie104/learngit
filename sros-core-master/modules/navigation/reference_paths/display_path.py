#from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

fig = plt.figure()
ax1 = fig.add_subplot(111)
#ax1.set_title('Fig')
plt.xlabel('x(m)')
plt.ylabel('y(m)')

data = numpy.loadtxt('/home/shen/Desktop/path0.dat')
x = []
y = []
for i in range(len(data)):
    x.append(data[i][0])
    y.append(data[i][1])
plt.plot(x, y, '.')

data = numpy.loadtxt('/home/shen/Desktop/path1.dat')
x = []
y = []
for i in range(len(data)):
    x.append(data[i][0])
    y.append(data[i][1])
plt.plot(x, y, '-')

x = []
y = []
x.append(-1)
y.append(-1)
x.append(25)
y.append(25)
plt.plot(x, y, '.')

plt.legend(['path0', 'path1'], loc = 0, ncol = 1)

plt.show()
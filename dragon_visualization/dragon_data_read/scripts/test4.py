import numpy
from pylab import * 

z = numpy.linspace(0,numpy.pi,100)
y = numpy.sin(z)
print y
plot(z, y)
show()

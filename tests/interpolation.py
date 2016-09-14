import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

x = np.linspace(-3, 3, 50)

y = [ 0.05743072, 0.00940577,  0.05848502, -0.16221342,  0.07314822,  0.08029109,
     -0.05064357,  0.0695457,   0.05350962,  0.03007541,  0.28596007,  0.05292537,
      0.20403795,  0.13377805,  0.30783861,  0.21393345,  0.403338,    0.37650354,
      0.5565254,   0.7653728,   0.73612424,  0.83256118,  0.96323466,  0.96258791,
      1.01120868,  0.86717364,  1.05338103,  0.99149619,  0.68564621,  0.76683369,
      0.63125403,  0.47328888,  0.37570884,  0.44782711,  0.30278194,  0.23449642,
     -0.00893587,  0.12843641, -0.07914852,  0.05046878,  0.03702803,  0.08291754,
      0.05008077, -0.09366895, -0.05902218,  0.19701947, -0.03468384, -0.06500214,
     -0.07205329,  0.2006148 ]

for i in range(1,11):
    spl = UnivariateSpline(x, y)
    plt.plot(x, y, 'ro', ms=5)
    smoothing_factor = i / 10.0
    print smoothing_factor
    spl.set_smoothing_factor(smoothing_factor)
    plt.plot(x, spl(x), 'g', lw=3)
    plt.show()

    

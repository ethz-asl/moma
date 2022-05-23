import matplotlib.pyplot as plt
import numpy as np

mu = 1e-2  # 1e-2 #100.0
delta = 1e-2  # 0.01 #0.01 #0.001

# value of the constraint
x = np.linspace(-1, 1, 5000)
print(min(x))
print(max(x))

p1 = -mu * np.log(delta) + mu * 0.5 * (((x[x <= delta] - 2 * delta) / delta) ** 2 - 1)
p2 = -mu * np.log(x[x > delta])
p = np.hstack((p1, p2))


plt.plot(x, p)
plt.grid(True)
plt.show()

import sys
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(threshold=sys.maxsize)
collect_data = np.loadtxt(sys.argv[1], delimiter=',')
measured = collect_data[collect_data[:,0]==1]
ts = measured[:,2]
ts_diff = np.diff(ts)
print(ts_diff[:50])
# plt.plot(ts_diff)
# plt.show()
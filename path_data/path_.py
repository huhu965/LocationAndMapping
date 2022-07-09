# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.backends.backend_pdf import PdfPages
# import numpy as np

# data_loam = np.loadtxt("./loam/optimize_trajectory/loam_odome_path.txt")
# data_loam_x = data_loam[:,3]
# data_loam_y = data_loam[:,7]
# data_loam_z = data_loam[:,11]

# data_lego = np.loadtxt("./lego_loam/optimize_trajectory/odome_path.txt")
# data_lego_x = data_lego[:,11]
# data_lego_y = data_lego[:,3]
# data_lego_z = data_lego[:,7]


# data_fast = np.loadtxt("./fast_lio/optimize_trajectory/odome_path.txt")
# data_fast_x = data_fast[:,3]
# data_fast_y = data_fast[:,7]
# data_fast_z = data_fast[:,11]

# data_RTKLio = np.loadtxt("./loam_frame/optimize_trajectory/odome_path.txt")
# data_RTKLio_x = data_RTKLio[:,3]
# data_RTKLio_y = data_RTKLio[:,7]
# data_RTKLio_z = data_RTKLio[:,11]

# data_truth = np.loadtxt("./loam/optimize_trajectory/loam_truth_path.txt")
# data_truth_x = data_truth[:,3]
# data_truth_y = data_truth[:,7]
# data_truth_z = data_truth[:,11]



# fig = plt.figure(figsize=(20,20))
# ax = Axes3D(fig)
# # ax.set_title("轨迹图")
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# ax.view_init(90,180)
# # ax.axis("off")

# # ax.set_ylim3d(ymin=-20,ymax=50)
# ax.set_zlim3d(zmin=-100,zmax=100)
# figure_loam = ax.plot(data_loam_x, data_loam_y, data_loam_z, c="y",linewidth=0.5, label="loam")
# figure_fast = ax.plot(data_fast_x, data_fast_y, data_fast_z, c="g",linewidth=0.5, label="fast_lio2")
# figure_RTKLio = ax.plot(data_RTKLio_x, data_RTKLio_y, data_RTKLio_z, c="r",linewidth=0.5, label="RTK_lio")
# figure_lego= ax.plot(data_lego_x, data_lego_y, data_lego_z, c="b",linewidth=0.5, label="lego-loam")
# figure_truth = ax.plot(data_truth_x, data_truth_y, data_truth_z, c="grey", linestyle="--", label="truth")

# plt.legend(loc=1)

# plt.savefig("test.pdf")
# plt.show()】

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

data_loam = np.loadtxt("./loam/optimize_trajectory/loam_odome_path.txt")
data_loam_x = data_loam[:,3]
data_loam_y = data_loam[:,7]
data_loam_z = data_loam[:,11]

data_lego = np.loadtxt("./lego_loam/optimize_trajectory/odome_path.txt")
data_lego_x = data_lego[:,11]
data_lego_y = data_lego[:,3]
data_lego_z = data_lego[:,7]


data_fast = np.loadtxt("./fast_lio/optimize_trajectory/odome_path.txt")
data_fast_x = data_fast[:,3]
data_fast_y = data_fast[:,7]
data_fast_z = data_fast[:,11]

data_RTKLio = np.loadtxt("./loam_frame/optimize_trajectory/odome_path.txt")
data_RTKLio_x = data_RTKLio[:,3]
data_RTKLio_y = data_RTKLio[:,7]
data_RTKLio_z = data_RTKLio[:,11]

data_truth = np.loadtxt("./loam/optimize_trajectory/loam_truth_path.txt")
data_truth_x = data_truth[:,3]
data_truth_y = data_truth[:,7]
data_truth_z = data_truth[:,11]



# plt.plot(data_loam_x, data_loam_y, c="y",linewidth=0.5, label="loam")
# plt.plot(data_fast_x+6, data_fast_y, c="g",linewidth=0.5, label="fast_lio2")
# plt.plot(data_RTKLio_x, data_RTKLio_y, c="r",linewidth=0.5, label="RTK_lio")
# plt.plot(data_lego_x+4.1, data_lego_y, c="b",linewidth=0.5, label="lego-loam")
# plt.plot(data_truth_x, data_truth_y, c="grey", linestyle="--", label="truth")

plt.plot(-data_loam_y, data_loam_x, c="y",linewidth=0.5, label="loam")
plt.plot(-data_fast_y, data_fast_x+6, c="g",linewidth=0.5, label="fast_lio2")
plt.plot(-data_RTKLio_y, data_RTKLio_x, c="r",linewidth=0.5, label="RTK_lio")
plt.plot(-data_lego_y, data_lego_x+4.1, c="b",linewidth=0.5, label="lego-loam")
plt.plot(-data_truth_y, data_truth_x, c="grey", linestyle="--", label="truth")
# plt.axis("off")
plt.legend(loc=1)

plt.savefig("test.pdf")
plt.show()
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("/home/ros/ros_ws/src/uvdar_ros_driver/test/uwb_kf_debug.csv")

plt.plot(df["t"].to_numpy(), df["truth"].to_numpy(), label="truth")
plt.plot(df["t"].to_numpy(), df["meas"].to_numpy(), label="meas", alpha=0.4)
plt.plot(df["t"].to_numpy(), df["est"].to_numpy(), label="filtered")

sigma = np.sqrt(df["var"].to_numpy())
plt.fill_between(
    df["t"].to_numpy(),
    df["est"].to_numpy() - 2 * sigma,
    df["est"].to_numpy() + 2 * sigma,
    alpha=0.2,
    label="±2σ"
)

plt.xlabel("time [s]")
plt.ylabel("range [m]")
plt.legend()
plt.grid(True)
plt.show()

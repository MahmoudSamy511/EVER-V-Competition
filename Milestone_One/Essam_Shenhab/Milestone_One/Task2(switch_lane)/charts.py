import pandas as pd
import matplotlib.pyplot as plt


def main():
    df1 = pd.read_csv("data/imu_data.csv")
    df2 = pd.read_csv("data/odom_data.csv")

    ref = pd.Timestamp(df1["Timestamp"][0])
    df1["Timestamp"] = df1["Timestamp"].apply(lambda x: (pd.Timestamp(x) - ref)).dt.total_seconds()

    ref = pd.Timestamp(df2["Timestamp"][0])
    df2["Timestamp"] = df2["Timestamp"].apply(lambda x: (pd.Timestamp(x) - ref)).dt.total_seconds()

    plt.figure(figsize=(12, 12))

    plt.subplot(2, 2, 1)
    plt.plot(df1['Timestamp'].values, df1['Linear_Acceleration_X'].values)
    plt.ylim(-3, 3)
    plt.xlabel('Time (seconds)')
    plt.ylabel('Linear Acceleration in X-Axis')
    plt.title('Linear Acceleration in X-Axis Over Time')

    plt.subplot(2, 2, 2)
    plt.plot(df1['Timestamp'].values, df1['Linear_Acceleration_Y'].values)
    plt.xlabel('Time (seconds)')
    plt.ylabel('Linear Acceleration in Y-Axis')
    plt.title('Linear Acceleration in Y-Axis Over Time')

    plt.subplot(2, 2, 3)
    plt.plot(df2['Timestamp'].values, df2['X'].values)
    # plt.ylim(-2, 2)
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position in X-Axis')
    plt.title('Position in X-Axis Over Time')

    plt.subplot(2, 2, 4)
    plt.plot(df2['Timestamp'].values, df2['Y'].values)
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position in Y-Axis')
    plt.title('Position in Y-Axis Over Time')

    plt.savefig("plots.png")


if __name__ == "__main__":
    main()
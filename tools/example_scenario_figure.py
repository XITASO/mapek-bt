"""Read in the logs from a scenario execution and display them."""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams["text.usetex"] = True


def filter_logs_by_timestamp(file_path, lower_threshold, upper_threshold):
    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Filter the DataFrame based on the timestamp column
    filtered_df = df[
        (df["timestamp"] > lower_threshold) & (df["timestamp"] < upper_threshold)
    ]
    filtered_df = filtered_df[(filtered_df["source"] == "/evaluator_log")]

    # Convert the filtered DataFrame back to a numpy array
    filtered_data = filtered_df.to_numpy()

    return filtered_data


def get_iou_data(df: pd.DataFrame, sliding_window_size: int = 5):
    timestamps = df[:, 0].astype(float) / 1000000000
    ious = df[:, 6].astype(float)
    
    # split the data into segments, where the time stamps are at most 0.2 seconds for two consecutive stamps
    segments = []
    current_segment_timestamps = [timestamps[0]]
    current_segment_ious = [ious[0]]

    # Using consecutive timestamp differences to segment data
    for i in range(1, len(timestamps)):
        if (timestamps[i] - timestamps[i - 1]) <= 0.5:
            current_segment_timestamps.append(timestamps[i])
            current_segment_ious.append(ious[i])
        else:
            segments.append((current_segment_timestamps, current_segment_ious))
            current_segment_timestamps = [timestamps[i]]
            current_segment_ious = [ious[i]]
    
    # Add the last segment
    segments.append((current_segment_timestamps, current_segment_ious))

    # Optionally, you could perform further analysis or return additional information 
    return segments, ious


if __name__ == "__main__":
    # Example usage
    file_path_baseline = "ros_ws/.data/logs/example_logs/example_baseline.csv"
    file_path_managing = "ros_ws/.data/logs/example_logs/example_with_mapek.csv"
    lower_threshold = 5
    upper_threshold = 45

    baseline_data = filter_logs_by_timestamp(
        file_path_baseline, lower_threshold * 1000000000, upper_threshold * 1000000000
    )
    segments_baseline, iou_baseline = get_iou_data(baseline_data)
    managing_data = filter_logs_by_timestamp(
        file_path_managing, lower_threshold * 1000000000, upper_threshold * 1000000000
    )
    segments_managing, iou_managing = get_iou_data(managing_data)

    events = [
        {"timestamp": 12.4, "desc": "image degraded"},
        {"timestamp": 13.4, "desc": "enhancement\n activated"},
        {"timestamp": 20, "desc": "flightphase descent", "offset_x": -1},
        {"timestamp": 20.7, "desc": "depth activated\nfuse rgb+depth"},
        {"timestamp": 33.4, "desc": "camera image dropped", "offset_y": 0.0},
        {"timestamp": 36.7, "desc": "camera redeployed", "offset_y": 0.0, "offset_x": -0.9},
        {"timestamp": 37.9, "desc": "camera activated", "offset_y": 0.0, "offset_x": -0.9},
    ]

    fig = plt.figure(figsize=(12, 3))
    c = plt.rcParams['axes.prop_cycle'].by_key()['color'][0]
    plt.plot(segments_baseline[0][0], segments_baseline[0][1], color=c, label="baseline")
    for t, iou in segments_baseline[1:]:
        plt.plot(t, iou, color=c)
    c = plt.rcParams['axes.prop_cycle'].by_key()['color'][1]
    plt.plot(segments_managing[0][0], segments_managing[0][1], color=c, label="managed")
    for t, iou in segments_managing[1:]:
        plt.plot(t, iou, color=c)

    for event in events:
        timestamp = event["timestamp"]
        desc = event["desc"]
        offset_y = 0 if not "offset_y" in event else event["offset_y"]
        offset_x = 0 if not "offset_x" in event else event["offset_x"]
        plt.axvline(x=timestamp, color="k", linestyle="--", linewidth=1)
        plt.text(
            x=timestamp + 0.2 + offset_x,
            y=max(max(iou_baseline), max(iou_managing)) - offset_y + 0.01,
            s=desc,
            rotation=90,
            verticalalignment="top",
            fontsize=12,
            color="black",
        )

    plt.legend()
    plt.xlim(lower_threshold, upper_threshold)
    plt.xlabel("time [s]")
    plt.ylabel("mIoU")

    plt.tight_layout()
    plt.savefig("./figures/example_scenario.svg")

from glob import glob
import pandas as pd
from system_interfaces.msg import AdaptationStatus
import numpy as np

def filter_logs_by_timestamp(df:pd.DataFrame, lower_threshold, upper_threshold):
    # Filter the DataFrame based on the timestamp column
    filtered_df = df[(df['timestamp'] > lower_threshold) & (df['timestamp'] < upper_threshold)]
    return filtered_df

def count_triggered_adaptations(df: pd.DataFrame):
    execution_rows = df[df['source'].str.contains('execute_lifecycle', na=False)]
    triggered_adaptations = execution_rows[execution_rows['adaptation_status'] == AdaptationStatus.STATUS_ADAPTATION_FINISHED]
    return len(triggered_adaptations)

def calculate_image_degraded_recovery_time(df: pd.DataFrame, filename: str):
    try:
        filtered_df = df[df['gt_failure_name'].str.contains('camera_degraded', na=False)]
    except:
        return 
    start = filtered_df.iloc[0]['timestamp']

    filtered_df = df[df['source'].str.contains('execute_comm_change/managed_subsystem/sensor_fusion', na=False)]
    finished_adaptation = filtered_df[filtered_df['adaptation_status'] ==  AdaptationStatus.STATUS_ADAPTATION_FINISHED]
    filtered_df = finished_adaptation[finished_adaptation['timestamp'] > start]
    successful_adaptation = filtered_df[filtered_df['success'] == 'True']

    if len(successful_adaptation) > 0:
        end = successful_adaptation.iloc[0]['timestamp']
        if (end-start) < 0:
            pass
        return (end - start) / 1e9
    else:
        return None

def calculate_redeploy_recovery_time(df: pd.DataFrame, filename: str):
    try:
        filtered_df = df[df['gt_failure_name'].str.contains('camera_image_dropped', na=False)]
    except:
        return
    start = filtered_df.iloc[0]['timestamp']

    filtered_df = df[df['source'].str.contains('execute_lifecycle/managed_subsystem/camera', na=False)]
    finished_adaptation = filtered_df[filtered_df['adaptation_status'] ==  AdaptationStatus.STATUS_ADAPTATION_FINISHED]
    successful_adaptation = finished_adaptation[finished_adaptation['success'] == 'True']

    # Find the first row with a timestamp greater than the given timestamp
    if len(successful_adaptation.index[successful_adaptation['timestamp'] > start] > 0):
        first_index = successful_adaptation.index[successful_adaptation['timestamp'] > start][0]

        # Extract the row with the found index
        redeploy_lc_change = successful_adaptation.loc[first_index]
        end = redeploy_lc_change['timestamp']
        recovery_time = (end - start) / 1e9
        if recovery_time < 0:
            pass
        return recovery_time
    else:
        return None

def evaluate_log_dir(log_dir: str):
    folders = glob(f'{log_dir}/*')

    recovery_times = dict()
    for scenario in folders:
        if 'activation_rules' in scenario:
            continue
        if not scenario in recovery_times:
            recovery_times[scenario] = list()
        triggered_adaptations = list()
        filenames = glob(scenario + '/*.csv')
        for filename in filenames:
            df = pd.read_csv(filename)
            start_stamp = df.iloc[0]['timestamp'] * 1e9
            filtered_data = filter_logs_by_timestamp(df, start_stamp, start_stamp+60*1e9) 
            num_triggered_adaptations = count_triggered_adaptations(filtered_data)

            triggered_adaptations.append(num_triggered_adaptations)

            if 'image_degradation' in scenario:
                time = calculate_image_degraded_recovery_time(filtered_data, filename)
                if time is None:
                    continue
                recovery_times[scenario].append(time)

            if 'message_drop' in scenario:
                time = calculate_redeploy_recovery_time(filtered_data, filename)
                if time is None:
                    continue
                recovery_times[scenario].append(time)

        print(f"For scenario {scenario}, there were {np.mean(triggered_adaptations)} \pm {round(np.std(triggered_adaptations), 2)} adaptations triggered")
        print(f"For scenario {scenario}, it took {round(np.mean(recovery_times[scenario]), 2)} \pm {round(np.std(recovery_times[scenario]), 2)} seconds to recover")

def main():
    evaluate_log_dir('./main')
    evaluate_log_dir('./wo_dep_graph_logs')


if __name__ == '__main__':
    main()

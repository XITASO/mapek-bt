import pandas as pd
from system_interfaces.msg import AdaptationStatus
from glob import glob
import numpy as np
from typing import Optional, List, Dict


def filter_logs_by_timestamp(
    df: pd.DataFrame, lower_threshold: float, upper_threshold: float
) -> pd.DataFrame:
    """
    Filter the DataFrame based on the timestamp column.

    Parameters:
    df (pd.DataFrame): The DataFrame containing log data.
    lower_threshold (float): The lower threshold for the timestamp.
    upper_threshold (float): The upper threshold for the timestamp.

    Returns:
    pd.DataFrame: The filtered DataFrame.
    """
    filtered_df = df[
        (df["timestamp"] > lower_threshold) & (df["timestamp"] < upper_threshold)
    ]
    return filtered_df


def calculate_success_rate(df: pd.DataFrame) -> Dict[str, Optional[Dict[str, int]]]:
    """
    Calculate the success rate of adaptations in the given DataFrame.

    Parameters:
    df (pd.DataFrame): The DataFrame containing log data.

    Returns:
    Dict[str, Optional[Dict[str, int]]]: A dictionary containing success rate statistics for each adaptation type.
    """

    def calculate_success_rate_adaptation_type(
        df: pd.DataFrame, adaptation_type: str
    ) -> Optional[Dict[str, int]]:
        """
        Calculate the success rate for a specific adaptation type.

        Parameters:
        df (pd.DataFrame): The DataFrame containing log data.
        adaptation_type (str): The type of adaptation to analyze.

        Returns:
        Optional[Dict[str, int]]: A dictionary containing statistics for the adaptation type, or None if no data exists.
        """
        execution_rows = df[df["source"].str.contains(adaptation_type, na=False)]
        if len(execution_rows) == 0:
            return None
        # Look for activations instead of redeploy for redeploy execution end
        if adaptation_type == "execute_redeploy":
            activation_rows = df[
                df["source"].str.contains("execute_lifecycle", na=False)
            ]
            # Pair up the execution starts and the successes
            finished_adaptations = activation_rows[
                activation_rows["adaptation_status"]
                == AdaptationStatus.STATUS_ADAPTATION_FINISHED
            ]
            finished_adaptations = finished_adaptations[
                finished_adaptations["success"] == "True"
            ]
            # Only get the trigger statements
            execution_rows = execution_rows[
                execution_rows["adaptation_status"]
                == AdaptationStatus.STATUS_ADAPTATION_TRIGGERED
            ]

            # Pair up the execution starts and the successes
            matched_count = 0
            durations = []
            for _, exe in execution_rows.iterrows():
                exe_timestamp = exe["timestamp"]
                closest_timestamp_row = (
                    finished_adaptations[
                        finished_adaptations["timestamp"] > exe_timestamp
                    ]
                    .sort_values(by="timestamp")
                    .head(1)
                )

                if not closest_timestamp_row.empty:
                    matched_count += 1
                    duration = closest_timestamp_row["timestamp"] - exe_timestamp
                    durations.append(duration)

            return {
                "successful_adaptations": matched_count,
                "triggered_adaptations": len(execution_rows),
            }

        else:
            finished_adaptations = execution_rows[
                execution_rows["adaptation_status"]
                == AdaptationStatus.STATUS_ADAPTATION_FINISHED
            ]

        # Count the number of rows with 'success' as True
        success_true_count = finished_adaptations[
            finished_adaptations["success"] == "True"
        ].shape[0]
        if finished_adaptations["success"].size > 10:
            return None
        if finished_adaptations["success"].size > 0:
            return {
                "successful_adaptations": success_true_count,
                "triggered_adaptations": finished_adaptations["success"].size,
            }
        else:
            return None

    success_rates = dict()
    triggered_num = 0
    for adaptation in [
        "execute_lifecycle",
        "execute_parametrization",
        "execute_comm_change",
        "execute_redeploy",
    ]:
        rate = calculate_success_rate_adaptation_type(df=df, adaptation_type=adaptation)
        if rate is not None:
            triggered_num += rate["triggered_adaptations"]
        success_rates[adaptation] = rate

    return success_rates


def calculate_mean_duration_time(
    df: pd.DataFrame, filename: str
) -> Dict[str, Optional[List[float]]]:
    """
    Calculate the mean duration time for adaptations in the given DataFrame.

    Parameters:
    df (pd.DataFrame): The DataFrame containing log data.
    filename (str): The name of the file (used for logging purposes).

    Returns:
    Dict[str, Optional[List[float]]]: A dictionary containing mean duration times for each adaptation type.
    """
    print(filename)

    def calculate_mean_time_adaptation(
        df: pd.DataFrame, adaptation_type: str
    ) -> Optional[List[float]]:
        """
        Calculate the mean time for a specific adaptation type.

        Parameters:
        df (pd.DataFrame): The DataFrame containing log data.
        adaptation_type (str): The type of adaptation to analyze.

        Returns:
        Optional[List[float]]: A list containing duration times for the adaptation type, or None if no data exists.
        """
        execution_rows = df[df["source"].str.contains(adaptation_type, na=False)]
        grouped_adaptations = execution_rows.groupby("source")
        durations = []
        for source, group in grouped_adaptations:
            finished_adaptations = group[
                group["adaptation_status"]
                == AdaptationStatus.STATUS_ADAPTATION_FINISHED
            ]
            triggered_adaptations = group[
                group["adaptation_status"]
                == AdaptationStatus.STATUS_ADAPTATION_TRIGGERED
            ]

            for (
                index_triggered,
                triggered_adaptation,
            ) in triggered_adaptations.iterrows():
                current_timestamp = triggered_adaptation["timestamp"]
                filtered_df = finished_adaptations[
                    finished_adaptations["timestamp"] > current_timestamp
                ]
                closest_row_3 = filtered_df.iloc[
                    (filtered_df["timestamp"] - current_timestamp).abs().argsort()[:1]
                ]
                duration = (
                    closest_row_3.iloc[0]["timestamp"]
                    - triggered_adaptation["timestamp"]
                )
                if duration < 0:
                    continue
                if duration == 0:
                    continue
                durations.append(duration)

        if len(durations) > 0:
            return durations
        else:
            return None

    def calculate_redeploy_duration_time(df: pd.DataFrame) -> Optional[List[float]]:
        """
        Calculate the mean duration time specifically for redeployment.

        Parameters:
        df (pd.DataFrame): The DataFrame containing log data.

        Returns:
        Optional[List[float]]: A list containing duration times for redeployment, or None if no data exists.
        """
        durations = []
        execution_rows = df[df["source"].str.contains("execute_redeploy", na=False)]
        if len(execution_rows) == 0:
            return None
        activation_rows = df[
            df["source"].str.contains(
                "execute_lifecycle/managed_subsystem/camera", na=False
            )
        ]
        finished_adaptations = activation_rows[
            activation_rows["adaptation_status"]
            == AdaptationStatus.STATUS_ADAPTATION_FINISHED
        ]
        finished_adaptations = finished_adaptations[
            finished_adaptations["success"] == "True"
        ]
        execution_rows = execution_rows[
            execution_rows["adaptation_status"]
            == AdaptationStatus.STATUS_ADAPTATION_TRIGGERED
        ]

        matched_count = 0
        durations = []
        for _, exe in execution_rows.iterrows():
            exe_timestamp = exe["timestamp"]
            closest_timestamp_row = (
                finished_adaptations[finished_adaptations["timestamp"] > exe_timestamp]
                .sort_values(by="timestamp")
                .head(1)
            )

            if not closest_timestamp_row.empty:
                matched_count += 1
                duration = closest_timestamp_row["timestamp"] - exe_timestamp
                durations.append(duration)

        if len(durations) > 0:
            return durations
        else:
            return None

    mean_duration_times = dict()
    for adaptation in [
        "execute_lifecycle",
        "execute_parametrization",
        "execute_comm_change",
    ]:
        mean_time = calculate_mean_time_adaptation(df=df, adaptation_type=adaptation)
        mean_duration_times[adaptation] = mean_time

    mean_duration_times["execute_redeploy"] = calculate_redeploy_duration_time(df=df)

    return mean_duration_times


def main() -> None:
    """
    Main function to analyze logs and compute adaptation statistics.
    """
    folders = glob("log_dump/*")

    success_rates = dict()
    mean_times = dict()

    to_delete = []

    for scenario in folders:
        filenames = glob(scenario + "/*.csv")
        run_delete = []
        for filename in filenames:
            df = pd.read_csv(filename)
            start_stamp = df.loc[0]["timestamp"] * 1e9
            filtered_data = filter_logs_by_timestamp(
                df, start_stamp, start_stamp + 60 * 1e9
            )
            rate = calculate_success_rate(filtered_data)
            for key, value in rate.items():
                if not key in success_rates:
                    success_rates[key] = list()
                success_rates[key].append(value)

            duration_times = calculate_mean_duration_time(filtered_data, filename)
            for key, value in duration_times.items():
                if not key in mean_times:
                    mean_times[key] = list()
                if value is not None:
                    mean_times[key].extend(value)

    for adaptation_type, adaptation_info in success_rates.items():
        successful_adaptations = 0
        triggered_adaptations = 0
        for item in adaptation_info:
            if item is None:
                continue
            successful_adaptations += item["successful_adaptations"]
            triggered_adaptations += item["triggered_adaptations"]

        if triggered_adaptations > 0:
            print(
                f"Success rate for {adaptation_type}: {successful_adaptations / triggered_adaptations}"
            )
            print(
                f"Number of adaptations for {adaptation_type}: suc: {successful_adaptations}, trigger: {triggered_adaptations}"
            )
        else:
            print(f"Success rate for {adaptation_type}: {0}")

    for adaptation_type, adaptation_info in mean_times.items():
        print(
            f"Mean duration for {adaptation_type}: {np.mean(adaptation_info) / 1e9} pm {np.std(adaptation_info) / 1e9}"
        )


if __name__ == "__main__":
    main()

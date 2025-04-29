"""Read in the logs from a scenario execution and compute statistics."""

import pandas as pd
import numpy as np
import os
from typing import Dict


def filter_logs_by_timestamp(df: pd.DataFrame, lower_threshold: float, upper_threshold: float) -> np.ndarray:
    """
    Filter the logs based on a timestamp threshold.

    Parameters:
    df (pd.DataFrame): The DataFrame containing log data.
    lower_threshold (float): The lower limit for the timestamp filter.
    upper_threshold (float): The upper limit for the timestamp filter.

    Returns:
    np.ndarray: The filtered logs as a numpy array.
    """
    # Filter the DataFrame based on the timestamp column
    filtered_df = df[
        (df["timestamp"] > lower_threshold) & (df["timestamp"] < upper_threshold)
    ]
    filtered_df = filtered_df[(filtered_df["source"] == "/evaluator_log")]

    # Convert the filtered DataFrame back to a numpy array
    filtered_data = filtered_df.to_numpy()

    return filtered_data


def analyze_single_run(file_path: str) -> Dict[str, float]:
    """
    Analyze the IoU from a single scenario run.

    Parameters:
    file_path (str): The path to the CSV file containing log data.

    Returns:
    Dict[str, float]: A dictionary containing IoU statistics of the scenario.
    """
    df = pd.read_csv(file_path)
    start_stamp = df["timestamp"][0] * 1e9
    filtered_data = filter_logs_by_timestamp(
        df, start_stamp + 5 * 1e9, start_stamp + 35 * 1e9
    )

    # Calculate the mean for the IoU columns, ignoring NaNs
    ious = {}
    for key, i in {"overall": 6, "vehicle": 7, "road": 8}.items():
        values = filtered_data[:, i].astype(float)
        mean = np.nanmean(values)
        # expected to see 300 messages in this time frame without any failures
        ious[key] = mean * len(values) / 300

    return ious


def pretty_print(result_dict: Dict[str, Dict[str, Dict[str, float]]]) -> None:
    """
    Pretty print the results given as a dictionary.

    Parameters:
    result_dict (Dict[str, Dict[str, Dict[str, float]]]): The dictionary containing run statistics.
    """
    for name in result_dict:
        print(name)
        for key in result_dict[name]:
            data = result_dict[name][key]
            print(f"\t{key}: {data['mean']:0.3}, {data['std']:0.3}")


def round_to_significant_figures(num: float, sig_figs: int = 3) -> str:
    """
    Round a number to a specific number of significant figures.

    Parameters:
    num (float): The number to be rounded.
    sig_figs (int): The number of significant figures to retain.

    Returns:
    str: The rounded number as a string.
    """
    if num == 0:
        return str(0)
    else:
        # Determine the number of digits before the decimal point
        num_digits = int(f"{num:e}".split('e')[1])  # Get exponent from scientific notation
        # Compute the shift needed to maintain significant figures
        shift = sig_figs - num_digits - 1
        str_num = str(round(num, shift))
        exp_chars = sig_figs if shift == 0 else sig_figs + 1
        if len(str_num) < exp_chars:
            str_num = str_num + "0"
        return str_num[:exp_chars]


def print_latex(result_dict: Dict[str, Dict[str, Dict[str, float]]]) -> None:
    """
    Print the results in LaTeX table format.

    Parameters:
    result_dict (Dict[str, Dict[str, Dict[str, float]]]): The dictionary containing run statistics.
    """
    # Define the mapping of scenarios to dictionary keys
    scenario_mapping = {
        "None & Present": "scenario_empty_full_rules",
        "F1 & Present": "scenario_image_degradation_full_rules",
        "F2 & Present": "scenario_message_drop_full_rules",
        "F1 \& S1 & Present": "scenario_F1+S1_full_rules",
        "None & Not present": "scenario_empty_activation_rules",
        "F1 & Not present": "scenario_image_degradation_activation_rules",
        "F2 & Not present": "scenario_message_drop_activation_rules",
        "F1 \& S1 & Not present": "scenario_F1+S1_activation_rules",
    }

    # Define a list representing the table's row info
    table_rows = [
        {"managing_system": "Not present", "scenario": "None"},
        {"managing_system": "Present", "scenario": "None"},
        {"managing_system": "Not present", "scenario": "F1"},
        {"managing_system": "Present", "scenario": "F1"},
        {"managing_system": "Not present", "scenario": "F2"},
        {"managing_system": "Present", "scenario": "F2"},
        {"managing_system": "Not present", "scenario": "F1 \& S1"},
        {"managing_system": "Present", "scenario": "F1 \& S1"},
    ]

    # Generate the LaTeX table filled with data
    latex_table = []

    for row in table_rows:
        managing_system = row["managing_system"]
        scenario = row["scenario"]
        key = f"{scenario} & {managing_system}"

        if key in scenario_mapping:
            data_key = scenario_mapping[key]
            mIoU_mean = result_dict[data_key]["overall"]["mean"]
            mIoU_std = result_dict[data_key]["overall"]["std"]

            IoU_road_mean = result_dict[data_key]["road"]["mean"]
            IoU_road_std = result_dict[data_key]["road"]["std"]

            IoU_vehicle_mean = result_dict[data_key]["vehicle"]["mean"]
            IoU_vehicle_std = result_dict[data_key]["vehicle"]["std"]

            # Format the values for LaTeX
            mIoU = f"{round_to_significant_figures(100*mIoU_mean)} $\pm$ {round_to_significant_figures(100*mIoU_std)}"
            IoU_road = f"{round_to_significant_figures(100*IoU_road_mean)} $\pm$ {round_to_significant_figures(100*IoU_road_std)}"
            IoU_vehicle = f"{round_to_significant_figures(100*IoU_vehicle_mean)} $\pm$ {round_to_significant_figures(100*IoU_vehicle_std)}"
        else:
            mIoU = "---"
            IoU_road = "---"
            IoU_vehicle = "---"

        # Add the formatted line to the LaTeX table
        latex_table.append(
            f"{scenario} & {managing_system} & {mIoU} & {IoU_road} & {IoU_vehicle} \\\\"
        )

    # Print the final LaTeX table
    print("\n".join(latex_table))


def main() -> None:
    """
    Main function to analyze scenario logs and generate summary statistics.
    """
    # get all scenarios we want to look at
    log_file_folder = "./log_dump"
    scenarios = []
    for s in [
        "scenario_image_degradation",
        "scenario_message_drop",
        "scenario_F1+S1",
        "scenario_empty",
    ]:
        for r in ["activation_rules", "full_rules"]:
            scenarios.append(s + "_" + r)

    # iterate through all scenarios
    overall_results = {}
    for scenario_folder in scenarios:
        runs_ious = []
        overall_results[scenario_folder] = {}
        for single_run in sorted(
            os.listdir(os.path.join(log_file_folder, scenario_folder))
        ):
            full_path = os.path.join(log_file_folder, scenario_folder, single_run)
            iou_dict = analyze_single_run(full_path)
            if any([np.isnan(val) for val in iou_dict.values()]):
                print(f"Warning: Nan result in file {full_path}")
            else:
                runs_ious.append(iou_dict)

        # analyze information across runs
        print(f"Found {len(runs_ious)} files to analyze for {scenario_folder}.")
        for key in ("overall", "vehicle", "road"):
            np_stats = np.array([val[key] for val in runs_ious])
            overall_results[scenario_folder][key] = {
                "mean": np.mean(np_stats),
                "std": np.std(np_stats),
            }

    print_latex(result_dict=overall_results)


if __name__ == "__main__":
    main()
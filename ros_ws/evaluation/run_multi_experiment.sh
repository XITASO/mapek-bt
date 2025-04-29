#!/bin/bash

# Number of repetitions
REPEAT=50

experiments=(
    scenario_image_degradation.yaml
    scenario_message_drop.yaml
    scenario_F1+S1.yaml
    scenario_empty.yaml
)

rule_files=(
    activation_rules.txt
    full_rules.txt
)

for i in $(seq 1 $REPEAT); do
    echo "Running experiment set $i..."

    # Run the experiment in a new session to isolate it
    for exp in "${experiments[@]}"; do
        for rules in "${rule_files[@]}"; do

            setsid ./ros_ws/evaluation/docker_single_experiment.sh $exp $rules
            echo "experiment $exp of set $i finished. Preparing for next experiment..."
            sleep 5

        done
    done
done

echo "All experiments completed."
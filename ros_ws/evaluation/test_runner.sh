#!/bin/bash

START=$((1 + $RANDOM % 250))

echo $START

python3 evaluation/create_random_timestamp.py $START

sleep 1

setsid ./evaluation/run_single_experiment.sh scenario_image_degraded.yaml $START
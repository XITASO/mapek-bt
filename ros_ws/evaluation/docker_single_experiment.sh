#!/bin/bash

#  set args
SCENARIO_FILE=empty 
if [ $# -gt 0 ]
  then
    SCENARIO_FILE=$1
fi

RULE_SET=rules.txt 
if [ $# -gt 0 ]
  then
    RULE_SET=$2
fi

docker build . -t mapek_bt -f ./Dockerfile.eval.cuda

echo "Build is done"

docker run -v ./logs:/logs --rm --name mapek_bt_evaluation mapek_bt evaluation/run_single_experiment.sh $SCENARIO_FILE $RULE_SET

echo "Fininshed docker run"
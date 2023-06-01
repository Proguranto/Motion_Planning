#!/bin/bash

# Define the command to run
command="python3 run.py -s 2dof_robot_arm -p rrtstar -o 2 --seed 0 -b 0.20"

# Define the eps values
bias_values=(0.05 0.2)

# Define the patterns to extract from the output
patterns=("cost")

for (( i=1; i<=5; i++ ))
do
  eval $command | grep "cost" >> data/rrtstar.txt
done
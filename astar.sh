#!/bin/bash

# Define the command to run
command="python run.py -s 3dof_robot_arm -p astar -o 2 --seed 0 -eps"

# Define the eps values
eps_values=(1 10 20)

# Define the patterns to extract from the output
patterns=("cost" "explored")

# Iterate over the eps values
for eps in "${eps_values[@]}"
do
  # Run the command with the current eps value and capture the output
  output=$(eval "$command $eps")

  # Iterate over the patterns and extract the corresponding lines from the output
  for pattern in "${patterns[@]}"
  do
    # Use grep to filter lines matching the pattern
    filtered_lines=$(echo "$output" | grep "$pattern")

    # Print the filtered lines
    echo "$filtered_lines" >> data/astar_3dof_o2_s0.txt
  done

  # Print a separator between iterations
  echo "============================"
done
#!/bin/bash -e

if rosmsg show robot_status; then
  echo "Robot Status are available"
else
  echo "Robot Status are not available"
  exit 1
fi


if rosmsg show MoveBaseActionGoal; then
  echo "Goal points are fed to Move Base"
else
  echo "Goal points are not fed to Move Base"
  exit 1
fi

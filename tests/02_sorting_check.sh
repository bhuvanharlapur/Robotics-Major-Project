#!/bin/bash -e

if rosmsg show GoalPoint; then
  echo "Goal Points are Found"
else
  echo "Goal Points are not found"
  exit 1
fi


if rosmsg show Header; then
  echo "Headers also found"
else
  echo "Headers are not found, Please check"
  exit 1
fi

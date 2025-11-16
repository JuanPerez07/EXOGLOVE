#!/usr/bin/env bash
set -euo pipefail

dir="with_relay"

# enable nullglob so the loop simply does nothing if no CSVs are found
shopt -s nullglob

for csv in "$dir"/*.csv; do
    # call the python script with the csv file and the 0 argument
    python3 plot.py "$csv" 0
done
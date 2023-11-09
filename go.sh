#!/bin/bash
set -em

show_help() {
  cat << EOF
Usage: ${0##*/} [-h] [-i ID] [-r RUNS] [-b BUILD] [-s SCENARIO] [-f FILE]

Run the FSW simulator.

  -h          Display this help text.
  -i ID       A base identifier for these runs.
  -r RUNS     Number of times to run the simulation. [Deafult: 1]
  -b BUILD    Build type for the FSW. [Default: release]
  -s SCENARIO Use a scenario configuration file.
  -f FILE     Use a pre-generated simulation file.
EOF
}

# Parse Args
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -h|--help) show_help; exit 0 ;;
        -i|--id) id=$2; shift ;;
        -r|--runs) runs=$2; shift ;;
        -b|--build) build="$2"; shift ;;
        -s|--scenario) scenario="$2"; shift ;;
        -f|--file) file="$2"; shift ;;
        *) echo "unexpected parameter: $1"; show_help; exit 1 ;;
    esac
    shift
done

# Option Defaults
runs="${runs:-1}"
build="${build:-release}"

if [ ! -d 42 ] && [ -z "$file" ]; then
   echo "Can't find 42 subdirectory; run git submodule init / update"
   exit 1
fi

# Build FSW
cargo build -p fsw "--$build"

# Run Simulation $repeitions Times
for i in $(seq 1 "$runs"); do
  if [ "$runs" -ne 1 ]; then
    echo "Run $i"
  fi

  if [ -n "$id" ]; then
    if [ "$runs" -ne 1 ]; then
      export MODALITY_RUN_ID="$id-$i"
    else
      export MODALITY_RUN_ID="$id"
    fi
  fi

  if [ -z "$file" ]; then
    (
      cd 42
      make -j8
      ./42 ../config/ &
    )
    pid42=$!
  fi

  target/"$build"/fsw ${scenario:+"--scenario=$scenario"} ${file:+"$file"}

  wait $pid42
  unset pid42
done

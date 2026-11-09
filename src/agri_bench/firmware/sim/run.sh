#!/bin/sh
# Build and run the host simulation of the stage firmware (needs g++).
set -e
cd "$(dirname "$0")"
out="${TMPDIR:-/tmp}/agri_bench_fw_sim"
g++ -std=c++17 -O1 -w -I. -include arduino_stub.h -x c++ sim.cpp -o "$out"
"$out"

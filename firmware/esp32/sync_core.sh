#!/bin/sh
# Copy the simulation-validated controller into the firmware library.
cd "$(dirname "$0")"
mkdir -p lib/botcore
cp ../../core/controller.hpp ../../core/controller.cpp ../../core/gains.hpp lib/botcore/
echo "lib/botcore updated from core/"

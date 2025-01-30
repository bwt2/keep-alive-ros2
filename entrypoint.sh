#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /app/keep-alive-test/install/setup.bash

exec "$@"
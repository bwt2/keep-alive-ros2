#!/bin/bash
set -e

source /opt/ros/humble/setup.sh
source /app/install/setup.sh

exec "$@"
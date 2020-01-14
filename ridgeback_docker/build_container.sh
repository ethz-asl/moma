#!/bin/sh
set -e
RIDGEBACK_DOCKER_DIR=$(dirname $(readlink -f "$0"))
docker build -t mobmi $RIDGEBACK_DOCKER_DIR
IMAGE_PATH=/tmp/mobmi_image.zip
docker save -o $IMAGE_PATH mobmi

echo "Saved container image to $IMAGE_PATH"


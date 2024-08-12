#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config.sh

rocker --x11 \
    --devices /dev/dri \
    --name=${CONTAINER_NAME} \
    --volume="${PROJECT_ROOT}:/waver_ws/src" \
    -- ${DOCKER_IMAGE_NAME}

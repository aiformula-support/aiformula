#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

docker build \
    -f ${SCRIPT_DIR}/aiformula_foxy_amd.dockerfile \
    -t aiformula:foxy_amd \
    ${SCRIPT_DIR}

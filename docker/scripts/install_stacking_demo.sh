#!/bin/bash
set -o pipefail

cd $MOMA_DEP_WS/src || exit 1

#pip install --no-cache-dir torch==1.12.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
pip install torch
sudo apt-get update
sudo apt-get install python3-tk


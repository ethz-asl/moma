#!/bin/bash

sudo apt-get update -y
sudo apt-get install python3-tk -y

pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip3 install pyransac3d imageio einops scikit-learn scikit-image tqdm torchmetrics lightgbm
pip3 install git+https://github.com/facebookresearch/segment-anything.git
pip3 install opencv-contrib-python
pip3 install numba gymnasium[extra] POT tensorflow stable-baselines3[extra]
# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*



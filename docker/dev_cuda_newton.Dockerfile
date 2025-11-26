# Base image: NVIDIA Isaac Sim
FROM nvcr.io/nvidia/isaac-sim:5.0.0

# Use bash for RUN commands
SHELL ["/bin/bash", "-c"]

# Install uv (Astral)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
# RUN /root/.local/bin/uv pip install "pyglet>=2.0" --system


# USAGE (it's a hack...)
# 1. Open a terminal on GPU machine in native system 
# 2. Run 'xhost +' in your native environment
# 3. From docker folder, run './run_isaac_newton_22.04.sh -b dev_cuda_newton.Dockerfile -w ~/Projects/sss_ral_ws/'
# 4. Inside docker container, run 'cd ~/moma_ws/src/newton/'
# 5. Run 'uv run -m newton.examples' to initialize uv-venv and print available examples
# 6. Run 'uv pip install "pyglet>=2.0"'

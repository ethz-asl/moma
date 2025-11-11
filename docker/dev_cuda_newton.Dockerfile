# Base image: NVIDIA Isaac Sim
FROM nvcr.io/nvidia/isaac-sim:5.0.0

# Use bash for RUN commands
SHELL ["/bin/bash", "-c"]

# Install uv (Astral)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
# RUN /root/.local/bin/uv pip install "pyglet>=2.0" --system

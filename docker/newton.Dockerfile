# Base image: NVIDIA Isaac Sim
FROM nvcr.io/nvidia/isaac-sim:5.0.0

# Use bash for RUN commands
SHELL ["/bin/bash", "-c"]

# Install uv (Astral)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# Install all the stuff for imgui-bundle
RUN apt-get update && apt-get install -y \
    build-essential \
    pkg-config \
    python3-dev \
    cmake \
    git \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    && rm -rf /var/lib/apt/lists/* # Clean up apt cache

# --- Entrypoint Integration ---
# 1. Copy the entrypoint script into the image
COPY newton_entrypoint.sh /usr/local/bin/newton_entrypoint.sh
RUN chmod +x /usr/local/bin/newton_entrypoint.sh

# 2. Set the ENTRYPOINT
ENTRYPOINT ["/usr/local/bin/newton_entrypoint.sh"]

# Set a default command to run if none is provided (e.g., launch a shell)
CMD ["/bin/bash"]
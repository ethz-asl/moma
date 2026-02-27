#!/bin/bash
# Enable logging (set -x) and error trapping (set -e) for safety
set -x 
set -e 

# Define the location of the mounted Newton source code
NEWTON_SRC="/root/moma_ws/src/newton"
VENV_DIR="$NEWTON_SRC/.venv"

# FIX APPLIED HERE: Using the correct confirmed path
UV_BIN="/root/.local/bin/uv"

export UV_HTTP_TIMEOUT=300

echo "--- ⚙️ Starting Newton Environment Setup ---"

# Change to the mounted source directory
cd "$NEWTON_SRC" || exit 1 # Fails loudly if directory isn't mounted

# 2. Create VENV if it doesn't exist
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating new virtual environment at $VENV_DIR..."
    "$UV_BIN" venv
else
    echo "Virtual environment already exists."
fi

# 3. FAST UV Sync: Ensures dependencies match the latest uv.lock
echo "Running fast UV sync for the latest dependencies..."
"$UV_BIN" sync --extra examples --extra sim --extra torch-cu12 --extra dev
    
# 4. Install additional, non-locked packages needed for the environment
echo "Installing required visualization extras (pyglet, Pillow, etc.)..."
"$UV_BIN" pip install "pyglet>=2.0" Pillow usd-core trimesh pycollada

# 5. Environment Activation (sets paths for the final shell)
export VIRTUAL_ENV="$VENV_DIR"
export PATH="$VENV_DIR/bin:$PATH"
echo "Environment activated. VENV PATH: $VENV_DIR"

echo "--- ✅ Newton Environment Ready ---"

# Execute the main command (e.g., /bin/bash) passed to the container
exec "$@"
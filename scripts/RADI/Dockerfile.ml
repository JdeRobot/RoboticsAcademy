FROM ra-base

ENV DEBIAN_FRONTEND=noninteractive

# Python ML dependencies only
RUN python3 -m pip install --upgrade pip

# Install PyTorch (GPU)
RUN python3 -m pip install --no-cache-dir \
    torch --extra-index-url https://download.pytorch.org/whl/cu128

# Optional lightweight libs (keep tight)
RUN python3 -m pip install --no-cache-dir \
    numpy \
    matplotlib

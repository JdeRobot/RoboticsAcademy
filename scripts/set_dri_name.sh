#!/bin/bash

# Function to get the GPU device path based on vendor priority
get_gpu_device() {
    preferred_vendor=$1
    # Default vendor priority: NVIDIA > Intel > others
    priority=(nvidia intel)

    # If a preferred vendor is provided, prioritize it
    if [ -n "$preferred_vendor" ]; then
        priority=("$preferred_vendor")
    fi

    # Get GPU information
    gpu_list=$(lspci -nn | grep VGA)

    for vendor in "${priority[@]}"; do
        if [[ "$vendor" == "nvidia" ]]; then
            # Check if nvidia-smi works
            if ! command -v nvidia-smi &> /dev/null || ! nvidia-smi &> /dev/null; then
                echo "Warning: nvidia-smi not available or failed, skipping NVIDIA GPU." >&2
                continue
            fi
        fi

        # Search for the vendor in the GPU list from lspci
        gpu_info=$(echo "$gpu_list" | grep -i "$vendor" | head -n 1)
        if [ -n "$gpu_info" ]; then
            bus=$(echo "$gpu_info" | cut -d' ' -f1)
            device=$(ls /sys/bus/pci/devices/0000:$bus/drm | grep card)
            if [ -n "$device" ]; then
                # Set the DRI_NAME environment variable to the card name (e.g., card0, card1)
                export DRI_NAME="$device"
                # Echo the selected vendor and DRI_NAME
                echo "Vendor: $vendor"
                echo "DRI_NAME: $DRI_NAME"
                return 0
            fi
        fi
    done

    echo "Error: No GPU found for the vendor '$preferred_vendor'." >&2
    return 1
}

# Check if a preferred vendor is passed as an argument
preferred_vendor="${1,,}"  # Convert to lowercase

# Get the GPU device path based on priority and set DRI_NAME
get_gpu_device "$preferred_vendor"

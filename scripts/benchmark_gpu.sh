#!/bin/sh

# Install glmark if needed
if [ -z "$(dpkg -l | grep glmark2)" ]; then
  apt update && apt install -y glmark2
fi

# Install pcitutils if needed
if [ -z "$(dpkg -l | grep pciutils)" ]; then
  apt install -y pciutils
fi

# Get the list of GPU PCI addresses and names using lspci (filter for VGA or 3D controllers)
gpu_list=$(lspci | grep -E "VGA|3D")

# Iterate over each GPU entry
echo "$gpu_list" | while IFS= read -r line; do
  # Extract the PCI address and GPU name
  pci=$(echo "$line" | awk '{print $1}')
  gpu_name=$(echo "$line" | awk -F ': ' '{print $2}' | awk '{for(i=1;i<NF;i++) printf "%s ", $i; print $NF}')

  # Find the corresponding /dev/dri/cardX device using PCI address
  gpu_path=$(ls /sys/bus/pci/devices/0000:$pci/drm | grep card)

  if [ -n "$gpu_path" ]; then
    gpu="/dev/dri/$gpu_path"

    # Display GPU and full name information
    echo -e "\nRunning glmark2 on GPU: $gpu (PCI: $pci, Name: $gpu_name)"

    # Run glmark2 with vglrun
    vglrun -d $gpu glmark2
  else
    echo -e "\nNo corresponding /dev/dri/card* found for PCI: $pci"
  fi
done

#!/bin/bash

# Function to get the GPU device path based on vendor priority
get_curr_version() {
    curr=$(pip show robotics_application_manager | egrep -o '([0-9]+\.){2}[0-9]+' | head -n 1)
    echo "Current version: $curr"
    return 0
}

get_running_version() {
    echo "Launched version: $(pip show robotics_application_manager | egrep -o '([0-9]+\.){2}[0-9]+' | head -n 1)"
    return 0
}

get_latest_version() {
    latest=$(pip index versions --retries 1 robotics_application_manager 2>/dev/null | egrep -o '([0-9]+\.){2}[0-9]+' | head -n 1)
    echo "Latest version: $latest"
    return 0
}


# Get the Robotics Application Manager and update it if needed
echo -e "\n--- Robotics Application Manager info ---"
get_curr_version
get_latest_version
if [[ "$curr" != "$latest" ]];then
    echo "Upgrading version:"
    python3 -m pip install --upgrade --retries 1 robotics_application_manager
else
    echo "Already on latest version"
fi
get_running_version
echo -e "-------------------------------------------\n"

#!/bin/bash

# Check if env.h exists
if [ ! -f "light-controller/env.h" ]; then
    # If env.h does not exist, copy env.h.example to env.h
    cp env.h.example light-controller/env.h
    # Print a message to the console instructing the user to update env.h
    echo "env.h was not found."
    echo "A new env.h file has been created from env.h.example."
    echo "Please update light-controller/env.h with your credentials."
else
    # If env.h already exists, do nothing
    echo "env.h already exists. No changes made."
fi

# Source your .env
# set -o allexport
# source .env # This line is commented out as env.h will be used directly
# set +o allexport

# Create env.h is now handled by the above conditional block
# cat > light-controller/env.h <<EOF
# #pragma once
#
# const char* WIFI_SSID = "${SSID}";
# const char* WIFI_PASS = "${WIFI_PASS}";
#
# const char* TAPO_USER = "${TAPO_EMAIL}";
# const char* TAPO_PASS = "${TAPO_PASS}";
# EOF

# echo "✅ env.h generated!" # This message is now conditional


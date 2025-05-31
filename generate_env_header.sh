#!/bin/bash

# Source your .env
set -o allexport
source .env
set +o allexport

# Create env.h
cat > light-controller/env.h <<EOF
#pragma once

const char* WIFI_SSID = "${SSID}";
const char* WIFI_PASS = "${WIFI_PASS}";

const char* TAPO_USER = "${TAPO_EMAIL}";
const char* TAPO_PASS = "${TAPO_PASS}";
EOF

echo "✅ env.h generated!"


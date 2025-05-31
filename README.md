# light-controller

## Configuration

Setting up your environment credentials is the first step to get the light controller working. This project uses a file named `env.h` located in the `light-controller` directory to store your WiFi and Tapo credentials.

**Important:** The `light-controller/env.h` file is intentionally not tracked by Git (it's listed in `.gitignore`) to prevent accidental exposure of your sensitive information.

You will find an example file named `env.h.example` in the root of the repository. This file serves as a template for `light-controller/env.h`.

**Setup Steps:**

1.  **Locate or Create `env.h`:**
    *   The script `generate_env_header.sh` is designed to help with this process. When you run it (e.g., as part of a build or setup process), it will check if `light-controller/env.h` exists.
    *   If `light-controller/env.h` does **not** exist, the script will automatically copy `env.h.example` to `light-controller/env.h` and print a message asking you to update it.
    *   If `light-controller/env.h` **already exists**, the script will inform you and make no changes.
    *   If you are not using the script, or if it fails, you can manually copy `env.h.example` from the root directory to the `light-controller` directory and rename it to `env.h`.

2.  **Update Credentials:**
    Open `light-controller/env.h` with a text editor and replace the placeholder values with your actual credentials:

    ```cpp
    #pragma once

    // WiFi Credentials
    #define WIFI_SSID "YOUR_ACTUAL_WIFI_SSID"
    #define WIFI_PASS "YOUR_ACTUAL_WIFI_PASSWORD"

    // Tapo Credentials
    #define TAPO_USER "YOUR_ACTUAL_TAPO_EMAIL"
    #define TAPO_PASS "YOUR_ACTUAL_TAPO_PASSWORD"
    ```

After completing these steps, the firmware will be able to use your credentials to connect to your WiFi network and control your Tapo device.

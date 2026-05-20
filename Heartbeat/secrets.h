#pragma once

/*
 *  IMPORTANT: Update the following constants with your WiFi and MQTT broker information before running the example.
 * - Group ID is used to separate the MQTT topics for different groups.
 * - Make sure you use your team number as the group ID (e.g., "1", "2", etc.) to avoid conflicts with other teams.
 * - Do not change the BROKER_HOST unless you have a different MQTT broker setup. 
 * - The default is set to a local broker on the lab network.
 */

#define WIFI_SSID "PhaseSpaceNetwork_2.4G"
#define WIFI_PASSWORD "8igMacNet"
#define BROKER_HOST "192.168.0.74"
#define BROKER_PORT 1883
#define GROUP_ID "1"


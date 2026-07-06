#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <FS.h>
#include <ArduinoJson.h>

// Maximum number of stored WiFi networks
#define MAX_WIFI_NETWORKS 5

struct WiFiCredential {
    char ssid[33];      // Max SSID length is 32 + null
    char password[65];  // Max WPA2 password length is 64 + null
};

// Settings class to manage configuration
class Settings {
public:
    // Multi-WiFi credentials
    WiFiCredential wifiNetworks[MAX_WIFI_NETWORKS];
    int wifiNetworkCount = 0;
    
    char dexcomUsername[50] = "";
    char dexcomPassword[50] = "";
    char libreViewEmail[50] = "";      // Added for LibreView
    char libreViewPassword[50] = "";   // Added for LibreView
    int serverRegion = 0;              // 0=US, 1=EU/Outside US, 2=Japan
    char libreViewRegion[8] = "";      // LibreView region (e.g. "", "eu", "eu2", "ap", "au", "de", "fr", "ca", "jp", "ae")
    char libreViewVersion[16] = "";    // Learned LibreLinkUp 'version' header (empty = use compiled default). Auto-updated if Abbott raises the minimum.
    char customTitle[30] = "Glucose Monitor"; // Default title
    char deviceName[24] = "";          // Optional user-facing name (shown at top of screen). Empty = hidden.
    int timeZoneOffset = -4;  // Default to Eastern Time (EDT, UTC-4)
    bool isDisplayInverted = true;  // Display inversion setting
    bool isDisplayRotated = false;   // Display rotation setting (180 degrees)
    int lowThreshold = 70;
    int highThreshold = 180;
    int criticalLowGlucoseValue = 0;  // Default disabled (0 = disabled)
    int criticalHighGlucoseValue = 0; // Default disabled (0 = disabled)
    int brightness = 255; // Display brightness (0-255, default 100%)
    bool disclaimerAccepted = false;  // Track if user accepted disclaimer

    // Add a WiFi network to stored list (deduplicates by SSID)
    bool addWifiNetwork(const char* ssid, const char* password) {
        if (!ssid || strlen(ssid) == 0) return false;
        
        // Check if SSID already exists - update password if so
        for (int i = 0; i < wifiNetworkCount; i++) {
            if (strcmp(wifiNetworks[i].ssid, ssid) == 0) {
                strncpy(wifiNetworks[i].password, password ? password : "", sizeof(wifiNetworks[i].password) - 1);
                wifiNetworks[i].password[sizeof(wifiNetworks[i].password) - 1] = '\0';
                Serial.printf("Updated WiFi network: %s\n", ssid);
                save();
                return true;
            }
        }
        
        // If full, remove oldest (index 0) and shift
        if (wifiNetworkCount >= MAX_WIFI_NETWORKS) {
            for (int i = 0; i < MAX_WIFI_NETWORKS - 1; i++) {
                wifiNetworks[i] = wifiNetworks[i + 1];
            }
            wifiNetworkCount = MAX_WIFI_NETWORKS - 1;
        }
        
        // Add new network at end
        strncpy(wifiNetworks[wifiNetworkCount].ssid, ssid, sizeof(wifiNetworks[wifiNetworkCount].ssid) - 1);
        wifiNetworks[wifiNetworkCount].ssid[sizeof(wifiNetworks[wifiNetworkCount].ssid) - 1] = '\0';
        strncpy(wifiNetworks[wifiNetworkCount].password, password ? password : "", sizeof(wifiNetworks[wifiNetworkCount].password) - 1);
        wifiNetworks[wifiNetworkCount].password[sizeof(wifiNetworks[wifiNetworkCount].password) - 1] = '\0';
        wifiNetworkCount++;
        
        Serial.printf("Added WiFi network: %s (%d/%d stored)\n", ssid, wifiNetworkCount, MAX_WIFI_NETWORKS);
        save();
        return true;
    }
    
    // Clear all stored WiFi networks
    void clearWifiNetworks() {
        wifiNetworkCount = 0;
        for (int i = 0; i < MAX_WIFI_NETWORKS; i++) {
            wifiNetworks[i].ssid[0] = '\0';
            wifiNetworks[i].password[0] = '\0';
        }
    }
    
    void load() {
        if (!SPIFFS.exists("/settings.json")) {
            Serial.println("Settings file not found");
            return;
        }
        
        File file = SPIFFS.open("/settings.json", FILE_READ);
        if (!file) {
            Serial.println("Failed to open settings file");
            return;
        }
        
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, file);
        
        if (error) {
            Serial.println("Failed to parse settings file");
        } else {
            strncpy(dexcomUsername, doc["dexcomUsername"] | "", sizeof(dexcomUsername) - 1);
            dexcomUsername[sizeof(dexcomUsername) - 1] = '\0';
            
            strncpy(dexcomPassword, doc["dexcomPassword"] | "", sizeof(dexcomPassword) - 1);
            dexcomPassword[sizeof(dexcomPassword) - 1] = '\0';
            
            // Handle migration from old useEUServer boolean to new serverRegion
            if (doc["serverRegion"].is<int>()) {
                serverRegion = doc["serverRegion"] | 0;
            } else if (doc["useEUServer"].is<bool>()) {
                // Migrate from old boolean format
                serverRegion = doc["useEUServer"].as<bool>() ? 1 : 0;
            } else {
                serverRegion = 0; // Default to US
            }
            // Handle migration from old useEULibreServer boolean to new libreViewRegion string
            if (doc["libreViewRegion"].is<const char*>()) {
                strncpy(libreViewRegion, doc["libreViewRegion"] | "", sizeof(libreViewRegion) - 1);
                libreViewRegion[sizeof(libreViewRegion) - 1] = '\0';
            } else if (doc["useEULibreServer"].is<bool>() && doc["useEULibreServer"].as<bool>()) {
                // Migrate from old boolean format
                strncpy(libreViewRegion, "eu", sizeof(libreViewRegion) - 1);
            } else {
                libreViewRegion[0] = '\0'; // Default to US (empty = US)
            }
            
            strncpy(libreViewEmail, doc["libreViewEmail"] | "", sizeof(libreViewEmail) - 1);
            libreViewEmail[sizeof(libreViewEmail) - 1] = '\0';
            
            strncpy(libreViewPassword, doc["libreViewPassword"] | "", sizeof(libreViewPassword) - 1);
            libreViewPassword[sizeof(libreViewPassword) - 1] = '\0';
            
            strncpy(libreViewVersion, doc["libreViewVersion"] | "", sizeof(libreViewVersion) - 1);
            libreViewVersion[sizeof(libreViewVersion) - 1] = '\0';
            
            strncpy(customTitle, doc["customTitle"] | "Glucose Monitor", sizeof(customTitle) - 1);
            customTitle[sizeof(customTitle) - 1] = '\0';
            
            strncpy(deviceName, doc["deviceName"] | "", sizeof(deviceName) - 1);
            deviceName[sizeof(deviceName) - 1] = '\0';
            
            timeZoneOffset = doc["timeZoneOffset"] | -4;  // Default to EDT
            isDisplayInverted = doc["isDisplayInverted"] | true;  // Default inverted
            isDisplayRotated = doc["isDisplayRotated"] | false;   // Default not rotated
            lowThreshold = doc["lowThreshold"] | 70;
            highThreshold = doc["highThreshold"] | 180;
            criticalLowGlucoseValue = doc["criticalLowGlucoseValue"] | 0;  // Default disabled
            criticalHighGlucoseValue = doc["criticalHighGlucoseValue"] | 0; // Default disabled
            brightness = doc["brightness"] | 255; // Default 100% brightness
            disclaimerAccepted = doc["disclaimerAccepted"] | false;  // Default not accepted
            
            // Load stored WiFi networks
            wifiNetworkCount = 0;
            if (doc["wifiNetworks"].is<JsonArray>()) {
                JsonArray wifiArray = doc["wifiNetworks"].as<JsonArray>();
                for (JsonObject net : wifiArray) {
                    if (wifiNetworkCount >= MAX_WIFI_NETWORKS) break;
                    strncpy(wifiNetworks[wifiNetworkCount].ssid, net["ssid"] | "", sizeof(wifiNetworks[wifiNetworkCount].ssid) - 1);
                    wifiNetworks[wifiNetworkCount].ssid[sizeof(wifiNetworks[wifiNetworkCount].ssid) - 1] = '\0';
                    strncpy(wifiNetworks[wifiNetworkCount].password, net["pass"] | "", sizeof(wifiNetworks[wifiNetworkCount].password) - 1);
                    wifiNetworks[wifiNetworkCount].password[sizeof(wifiNetworks[wifiNetworkCount].password) - 1] = '\0';
                    if (strlen(wifiNetworks[wifiNetworkCount].ssid) > 0) {
                        wifiNetworkCount++;
                    }
                }
            }
            
            Serial.println("Settings loaded successfully");
            Serial.print("Dexcom Username length: ");
            Serial.println(strlen(dexcomUsername));
            Serial.print("Dexcom Password length: ");
            Serial.println(strlen(dexcomPassword));
            Serial.print("Server Region: ");
            Serial.println(serverRegion == 0 ? "US" : (serverRegion == 1 ? "EU/Outside US" : "Japan"));
            Serial.print("LibreView Email length: ");
            Serial.println(strlen(libreViewEmail));
            Serial.print("LibreView Password length: ");
            Serial.println(strlen(libreViewPassword));
            Serial.print("Custom Title: ");
            Serial.println(customTitle);
            Serial.print("Time Zone Offset: UTC");
            Serial.println(timeZoneOffset);
            Serial.print("Display Inverted: ");
            Serial.println(isDisplayInverted ? "Yes" : "No");
            Serial.print("Display Rotated: ");
            Serial.println(isDisplayRotated ? "Yes" : "No");
            Serial.print("Low Threshold: ");
            Serial.println(lowThreshold);
            Serial.print("High Threshold: ");
            Serial.println(highThreshold);
            Serial.print("Stored WiFi networks: ");
            Serial.println(wifiNetworkCount);
            for (int i = 0; i < wifiNetworkCount; i++) {
                Serial.printf("  [%d] %s\n", i, wifiNetworks[i].ssid);
            }
        }
        
        file.close();
    }

    void save() {
        File file = SPIFFS.open("/settings.json", FILE_WRITE);
        if (!file) {
            Serial.println("Failed to open settings file for writing");
            return;
        }
        
        JsonDocument doc;
        doc["dexcomUsername"] = dexcomUsername;
        doc["dexcomPassword"] = dexcomPassword;
        doc["serverRegion"] = serverRegion;
        doc["libreViewRegion"] = libreViewRegion; // LibreView region string (e.g. "eu", "ap", "")
        doc["libreViewEmail"] = libreViewEmail;      // Save LibreView email
        doc["libreViewPassword"] = libreViewPassword; // Save LibreView password
        doc["libreViewVersion"] = libreViewVersion;   // Save learned LibreLinkUp version header
        doc["customTitle"] = customTitle;
        doc["deviceName"] = deviceName;
        doc["timeZoneOffset"] = timeZoneOffset;
        doc["isDisplayInverted"] = isDisplayInverted;
        doc["isDisplayRotated"] = isDisplayRotated;
        doc["lowThreshold"] = lowThreshold;
        doc["highThreshold"] = highThreshold;
        doc["criticalLowGlucoseValue"] = criticalLowGlucoseValue;
        doc["criticalHighGlucoseValue"] = criticalHighGlucoseValue;
        doc["brightness"] = brightness;
        doc["disclaimerAccepted"] = disclaimerAccepted;
        
        // Save stored WiFi networks
        JsonArray wifiArray = doc["wifiNetworks"].to<JsonArray>();
        for (int i = 0; i < wifiNetworkCount; i++) {
            JsonObject net = wifiArray.add<JsonObject>();
            net["ssid"] = wifiNetworks[i].ssid;
            net["pass"] = wifiNetworks[i].password;
        }
        
        if (serializeJson(doc, file) == 0) {
            Serial.println("Failed to write settings");
        } else {
            Serial.println("Settings saved successfully");
        }
        
        file.close();
    }

    void reset() {
        dexcomUsername[0] = '\0';
        dexcomPassword[0] = '\0';
        libreViewEmail[0] = '\0';      // Reset LibreView email
        libreViewPassword[0] = '\0';   // Reset LibreView password
        serverRegion = 0;             // Reset to US server
        libreViewRegion[0] = '\0';    // Reset LibreView region to US
        libreViewVersion[0] = '\0';   // Reset learned version (compiled default will be used / re-learned)
        strncpy(customTitle, "Glucose Monitor", sizeof(customTitle) - 1);
        customTitle[sizeof(customTitle) - 1] = '\0';
        deviceName[0] = '\0';
        timeZoneOffset = -4;  // Reset to EDT
        isDisplayInverted = true;  // Reset inversion setting
        isDisplayRotated = false;   // Reset rotation setting
        lowThreshold = 70;
        highThreshold = 180;
        criticalLowGlucoseValue = 0;  // Reset to disabled
        criticalHighGlucoseValue = 0; // Reset to disabled
        disclaimerAccepted = false;   // Reset disclaimer acceptance
        clearWifiNetworks();
        save();
    }

    void debug() {
        Serial.println("=== Settings Debug ===");
        Serial.print("Dexcom Username: ");
        Serial.println(dexcomUsername);
        Serial.print("Dexcom Password length: ");
        Serial.println(strlen(dexcomPassword));
        Serial.print("Server Region: ");
        Serial.println(serverRegion == 0 ? "US" : (serverRegion == 1 ? "EU/Outside US" : "Japan"));
        Serial.print("LibreView Email length: ");
        Serial.println(strlen(libreViewEmail));
        Serial.print("LibreView Password length: ");
        Serial.println(strlen(libreViewPassword));
        Serial.print("Custom Title: ");
        Serial.println(customTitle);
        Serial.print("Time Zone Offset: UTC");
        Serial.println(timeZoneOffset);
        Serial.print("Display Inverted: ");
        Serial.println(isDisplayInverted ? "Yes" : "No");
        Serial.print("Display Rotated: ");
        Serial.println(isDisplayRotated ? "Yes" : "No");
        Serial.print("Low Threshold: ");
        Serial.println(lowThreshold);
        Serial.print("High Threshold: ");
        Serial.println(highThreshold);
        Serial.print("Critical Low Value: ");
        Serial.println(criticalLowGlucoseValue);
        Serial.print("Critical High Value: ");
        Serial.println(criticalHighGlucoseValue);
        Serial.println("======================");
    }
};

#endif // SETTINGS_H

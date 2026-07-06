#ifndef DEXCOM_CLIENT_H
#define DEXCOM_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include "Settings.h"

// Dexcom API URLs - US Server
#define US_AUTHENTICATE_URL "https://share2.dexcom.com/ShareWebServices/Services/General/AuthenticatePublisherAccount"
#define US_LOGIN_URL "https://share2.dexcom.com/ShareWebServices/Services/General/LoginPublisherAccountById"
#define US_GLUCOSE_URL "https://share2.dexcom.com/ShareWebServices/Services/Publisher/ReadPublisherLatestGlucoseValues"
// Dexcom API URLs - EU/Outside US Server
#define EU_AUTHENTICATE_URL "https://shareous1.dexcom.com/ShareWebServices/Services/General/AuthenticatePublisherAccount"
#define EU_LOGIN_URL "https://shareous1.dexcom.com/ShareWebServices/Services/General/LoginPublisherAccountById"
#define EU_GLUCOSE_URL "https://shareous1.dexcom.com/ShareWebServices/Services/Publisher/ReadPublisherLatestGlucoseValues"
// Dexcom API URLs - Japan Server
#define JP_AUTHENTICATE_URL "https://shareous1.dexcom.jp/ShareWebServices/Services/General/AuthenticatePublisherAccount"
#define JP_LOGIN_URL "https://shareous1.dexcom.jp/ShareWebServices/Services/General/LoginPublisherAccountById"
#define JP_GLUCOSE_URL "https://shareous1.dexcom.jp/ShareWebServices/Services/Publisher/ReadPublisherLatestGlucoseValues"
#define DEFAULT_APP_ID "d8665ade-9673-4e27-9ff6-92db4ce13d13"

// Server region constants
#define SERVER_REGION_US 0
#define SERVER_REGION_EU 1
#define SERVER_REGION_JP 2

// Forward declarations
extern Settings settings;
extern TFT_eSPI tft;

// Diagnostic bridge (avoid direct dependency on DisplayManager)
// Keep ordering consistent with DisplayManager::DiagnosticState in the main sketch
enum DiagnosticCode {
    DIAG_OK = 0,
    DIAG_WIFI_DISCONNECTED = 1,
    DIAG_MISSING_CREDENTIALS = 2,
    DIAG_AUTH_FAILED = 3,
    DIAG_LOGIN_FAILED = 4,
    DIAG_SESSION_EXPIRED = 5,
    DIAG_SERVER_ERROR = 6,
    DIAG_CONNECTION_ERROR = 7,
    DIAG_DATA_STALE = 8
};

// Implemented in the main sketch (Dexcom-v0.1.ino)
extern void setDiagnosticStatePublicInt(int state);

// Glucose reading structure
struct GlucoseReading {
    float value;
    unsigned long timestamp;
    char trend[20];
    unsigned long long dexcomTime;
};

// Note: MAX_GLUCOSE_HISTORY is defined in the main .ino file

// HTTP timeout for all requests
#define HTTP_TIMEOUT_MS 8000

class DexcomClient {
private:
    char accountId[100] = "";
    char sessionId[100] = "";
    int lastHttpError = 0;  // Track last HTTP error code for backoff logic
public:
    int getLastHttpError() { return lastHttpError; }
    void clearLastHttpError() { lastHttpError = 0; }
private:
    String authenticateUrl;
    String loginUrl;
    String glucoseUrl;
    const char* applicationId;
    
    
public:
    DexcomClient() : applicationId(DEFAULT_APP_ID) {}
    
    void init() {
        // Set URLs based on server region selection
        switch (settings.serverRegion) {
            case SERVER_REGION_EU:
                authenticateUrl = EU_AUTHENTICATE_URL;
                loginUrl = EU_LOGIN_URL;
                glucoseUrl = EU_GLUCOSE_URL;
                Serial.println("Using EU Dexcom servers (Outside US)");
                break;
            case SERVER_REGION_JP:
                authenticateUrl = JP_AUTHENTICATE_URL;
                loginUrl = JP_LOGIN_URL;
                glucoseUrl = JP_GLUCOSE_URL;
                Serial.println("Using Japan Dexcom servers");
                break;
            case SERVER_REGION_US:
            default:
                authenticateUrl = US_AUTHENTICATE_URL;
                loginUrl = US_LOGIN_URL;
                glucoseUrl = US_GLUCOSE_URL;
                Serial.println("Using US Dexcom servers");
                break;
        }
    }
    
    bool authenticate(bool showUI = true) {
        if (WiFi.status() != WL_CONNECTED) {
            return false;
        }
        
        // Check for valid credentials
        if (strlen(settings.dexcomUsername) == 0 || strlen(settings.dexcomPassword) == 0) {
            Serial.println("Error: Dexcom credentials are empty!");
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_RED);
            tft.setCursor(20, 60);
            tft.println("Missing Dexcom credentials");
            tft.setCursor(20, 90);
            tft.println("Reset & reconfigure");
            // Set diagnostic state for missing credentials
            setDiagnosticStatePublicInt(DIAG_MISSING_CREDENTIALS);
            return false;
        }
        
        HTTPClient http;
        Serial.println("Authenticating to Dexcom (step 1)...");
        Serial.printf("Using URL: %s\n", authenticateUrl.c_str());
        Serial.printf("Username length: %d\n", strlen(settings.dexcomUsername));
        Serial.print("Using App ID: ");
        Serial.println(applicationId);
        
        if (showUI) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Explicitly set white text
            tft.setCursor(10, 60);  // Move further left
            tft.println("Connecting to");
            tft.setCursor(10, 90);  // Position for second line
            tft.println("Dexcom...");
        }
        
        // Use stack-allocated client for each request
        WiFiClientSecure client;
        client.setInsecure(); // Skip certificate validation
        
        if (!http.begin(client, authenticateUrl.c_str())) {
            Serial.println("Failed to set up HTTPS connection");
            return false;
        }
        
        // Add headers that mimic a browser
        http.addHeader("Content-Type", "application/json");
        http.addHeader("User-Agent", "Dexcom Share/3.0.2.11 CFNetwork/711.2.23 Darwin/14.0.0");
        http.addHeader("Accept", "application/json");
        http.setTimeout(HTTP_TIMEOUT_MS);
        
        // Create authentication payload
        char authenticatePayload[256];
        snprintf(authenticatePayload, sizeof(authenticatePayload), 
                 "{\"accountName\":\"%s\",\"password\":\"%s\",\"applicationId\":\"%s\"}", 
                 settings.dexcomUsername, settings.dexcomPassword, applicationId);
        
        Serial.println("Sending authentication request...");
        Serial.print("Payload size: ");
        Serial.println(strlen(authenticatePayload));
        yield(); // Allow other tasks (like touch input) to process
        
        int httpResponseCode = http.POST(authenticatePayload);
        String response = http.getString();
        
        if (httpResponseCode == HTTP_CODE_OK) {
            // Remove quotes from account ID
            response.replace("\"", "");
            strncpy(accountId, response.c_str(), sizeof(accountId) - 1);
            accountId[sizeof(accountId) - 1] = '\0';
            
            Serial.print("Account ID: ");
            Serial.println(accountId);
            
            http.end();
            return true;
        } else {
            Serial.print("Error on authentication: ");
            Serial.println(httpResponseCode);
            Serial.print("Response: ");
            Serial.println(response);
            
            // Display detailed error
            if (showUI) {
                tft.fillScreen(TFT_BLACK);
                tft.setCursor(20, 30);
                tft.println("Auth Error: " + String(httpResponseCode));
                
                tft.setTextSize(1);
                tft.setCursor(20, 60);
                tft.println("Response: " + response.substring(0, 120));
                
                tft.setCursor(20, 80);
                if (httpResponseCode == 500) {
                    tft.println("Server error. Try these fixes:");
                    tft.setCursor(20, 90);
                    tft.println("1. Reset WiFi & Dexcom credentials");
                    tft.setCursor(20, 100);
                    tft.println("2. Try the other server (US/EU)");
                    tft.setCursor(20, 110);
                    tft.println("3. Check Dexcom Share is enabled");
                    tft.setCursor(20, 120);
                    tft.println("4. Restart your device");
                    // Show current server region
                    tft.setCursor(20, 140);
                    tft.setTextSize(2);
                    const char* regionName = settings.serverRegion == 0 ? "US" : 
                                            (settings.serverRegion == 1 ? "EU" : "Japan");
                    tft.printf("Current server: %s", regionName);
                } else if (httpResponseCode == -1) {
                    tft.println("Connection failed");
                    tft.setCursor(20, 90);
                    tft.println("Check internet & try again");
                } else {
                    tft.println(http.errorToString(httpResponseCode));
                }
            }
            if (httpResponseCode == 500) {
                setDiagnosticStatePublicInt(DIAG_SERVER_ERROR);
            } else if (httpResponseCode == -1) {
                setDiagnosticStatePublicInt(DIAG_CONNECTION_ERROR);
            } else {
                setDiagnosticStatePublicInt(DIAG_AUTH_FAILED);
            }
            
            accountId[0] = '\0'; // Reset on failure
            http.end();
            delay(2000);  // Shorter delay
            return false;
        }
    }
    
    bool authenticateWithRetry(bool showUI = true) {
        for (int attempt = 0; attempt < 3; attempt++) {
            if (authenticate(showUI)) {
                return true;
            }
            Serial.printf("Authentication attempt %d failed, retrying...\n", attempt + 1);
            delay(1000);
        }
        return false;
    }
    
    bool login(bool showUI = true) {
        if (WiFi.status() != WL_CONNECTED || strlen(accountId) == 0) {
            return false;
        }
        
        HTTPClient http;
        Serial.println("Logging in to Dexcom (step 2)...");
        
        // Use stack-allocated client for each request
        WiFiClientSecure client;
        client.setInsecure(); // Skip certificate validation
        
        if (!http.begin(client, loginUrl.c_str())) {
            Serial.println("Failed to set up HTTPS connection");
            return false;
        }
        
        http.addHeader("Content-Type", "application/json");
        http.addHeader("User-Agent", "Dexcom Share/3.0.2.11 CFNetwork/711.2.23 Darwin/14.0.0");
        http.addHeader("Accept", "application/json");
        http.setTimeout(HTTP_TIMEOUT_MS);
        
        // Create login payload
        char loginPayload[256];
        snprintf(loginPayload, sizeof(loginPayload), 
                 "{\"accountId\":\"%s\",\"password\":\"%s\",\"applicationId\":\"%s\"}", 
                 accountId, settings.dexcomPassword, applicationId);
        
        Serial.println("Sending login request...");
        yield(); // Allow other tasks (like touch input) to process
        
        int httpResponseCode = http.POST(loginPayload);
        String response = http.getString();
        
        if (httpResponseCode == HTTP_CODE_OK) {
            // Remove quotes from session ID
            response.replace("\"", "");
            strncpy(sessionId, response.c_str(), sizeof(sessionId) - 1);
            sessionId[sizeof(sessionId) - 1] = '\0';
            
            Serial.print("Session ID: ");
            Serial.println(sessionId);
            
            http.end();
            return true;
        } else {
            Serial.print("Error on login: ");
            Serial.println(httpResponseCode);
            Serial.print("Response: ");
            Serial.println(response);
            
            // Display error
            if (showUI) {
                tft.fillScreen(TFT_BLACK);
                tft.setCursor(20, 60);
                tft.println("Login Error: " + String(httpResponseCode));
                
                tft.setTextSize(1);
                tft.setCursor(20, 90);
                tft.println("Response: " + response.substring(0, 60));
            }
            
            // Set diagnostic state for login failure
            setDiagnosticStatePublicInt(DIAG_LOGIN_FAILED);

            sessionId[0] = '\0'; // Reset on failure
            http.end();
            return false;
        }
    }
    
    bool fetchGlucoseData(float& current_glucose, float& previous_glucose, 
                          char* trend, char* timestamp, unsigned long long& lastDexcomTime,
                          bool isOverdueCheck = false) {
        if (WiFi.status() != WL_CONNECTED || strlen(sessionId) == 0) {
            return false;
        }
        
        // Retry once on transient errors (timeout, connection reset, etc.)
        const int maxAttempts = 2;
        for (int attempt = 1; attempt <= maxAttempts; attempt++) {
            if (attempt > 1) {
                Serial.printf("Retry attempt %d/%d after transient error...\n", attempt, maxAttempts);
                delay(2000); // Brief pause before retry
                yield();
                // Re-check WiFi before retry
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.println("WiFi lost during retry - aborting");
                    return false;
                }
            }
            
            HTTPClient http;
            
            // Use stack-allocated client for each request
            WiFiClientSecure client;
            client.setInsecure(); // Skip certificate validation
            
            // Build URL
            char url[256];
            snprintf(url, sizeof(url), "%s?sessionId=%s&minutes=1440&maxCount=2", 
                     glucoseUrl.c_str(), sessionId);
            
            if (!http.begin(client, url)) {
                Serial.println("Failed to set up HTTPS connection");
                http.end();
                continue; // Retry
            }
            
            http.addHeader("Content-Type", "application/json");
            http.addHeader("User-Agent", "Dexcom Share/3.0.2.11 CFNetwork/711.2.23 Darwin/14.0.0");
            http.addHeader("Accept", "application/json");
            http.setTimeout(HTTP_TIMEOUT_MS);
            yield(); // Allow other tasks (like touch input) to process
            
            int httpResponseCode = http.GET();
            
            if (httpResponseCode == HTTP_CODE_OK) {
                String response = http.getString();
                Serial.print("Glucose response: ");
                Serial.println(response);
                
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, response);
                
                if (!error && doc.is<JsonArray>() && doc.size() > 0) {
                    JsonObject reading = doc[0];
                    
                    current_glucose = reading["Value"].as<float>();
                    strncpy(trend, reading["Trend"].as<const char*>(), 19);
                    trend[19] = '\0';
                    
                    // Get the timestamp and parse it
                    String timestampStr = reading["WT"].as<String>();
                    
                    // Extract just the timestamp part - handle both Date( and /Date( formats
                    int openParenIndex = timestampStr.indexOf('(');
                    int closeParenIndex = timestampStr.indexOf(')');
                    
                    if (openParenIndex != -1 && closeParenIndex != -1) {
                        // Extract timestamp between parentheses, ignoring timezone suffix after +/- 
                        String timestampOnly = timestampStr.substring(openParenIndex + 1, closeParenIndex);
                        // Handle both positive (+0500) and negative (-0500) timezone offsets
                        int dashIndex = timestampOnly.indexOf('-');
                        int plusIndex = timestampOnly.indexOf('+');
                        int tzIndex = -1;
                        if (dashIndex != -1) tzIndex = dashIndex;
                        if (plusIndex != -1 && (tzIndex == -1 || plusIndex < tzIndex)) tzIndex = plusIndex;
                        if (tzIndex != -1) {
                            timestampOnly = timestampOnly.substring(0, tzIndex); // Remove timezone part
                        }
                        timestampStr = timestampOnly;
                    } else {
                        Serial.println("Error: Could not parse timestamp format");
                        http.end();
                        return false; // Don't retry on parse errors - same response would fail again
                    }
                    
                    strncpy(timestamp, timestampStr.c_str(), 9);
                    timestamp[9] = '\0';
                    
                    // Parse the full timestamp for comparison
                    char* endptr;
                    lastDexcomTime = strtoull(timestampStr.c_str(), &endptr, 10);
                    
                    // Debug timestamp parsing
                    Serial.print("Parsed timestamp: ");
                    Serial.print(timestampStr);
                    Serial.print(" -> ");
                    Serial.println(lastDexcomTime);
                    
                    // Get previous glucose if available
                    if (doc.size() > 1) {
                        JsonObject previousReading = doc[1];
                        previous_glucose = previousReading["Value"].as<float>();
                        Serial.printf("API returned %d readings - using previous reading: %.1f\n", doc.size(), previous_glucose);
                    } else {
                        // IMPORTANT: Don't overwrite previous_glucose with current_glucose!
                        // Preserve the existing previous_glucose value for accurate differential calculation
                        // However, if previous_glucose is 0 (uninitialized), set it to current_glucose to avoid huge differential on startup
                        if (previous_glucose == 0) {
                            previous_glucose = current_glucose;
                            Serial.printf("API returned only %d reading and previous_glucose was 0 (startup) - setting to current: %.1f\n", doc.size(), previous_glucose);
                        } else {
                            Serial.printf("API returned only %d reading - preserving previous_glucose: %.1f\n", doc.size(), previous_glucose);
                        }
                    }
                    
                    Serial.printf("Current glucose: %.1f, Previous: %.1f, Trend: %s\n", 
                                  current_glucose, previous_glucose, trend);
                    
                    http.end();
                    return true;
                } else {
                    Serial.print("JSON parsing error: ");
                    Serial.println(error.c_str());
                    http.end();
                    return false; // Don't retry on JSON errors - same response would fail again
                }
            } else {
                Serial.printf("HTTP error: %d (attempt %d/%d)\n", httpResponseCode, attempt, maxAttempts);
                lastHttpError = httpResponseCode;  // Track for backoff logic
                
                // Reset session credentials on 500 server errors
                if (httpResponseCode == 500) {
                    Serial.println("Server error (500) detected - resetting session credentials");
                    sessionId[0] = '\0';
                    accountId[0] = '\0';
                    http.end();
                    return false; // Don't retry - need re-auth
                }
                
                // Don't retry on 429 rate limiting - back off instead
                if (httpResponseCode == 429) {
                    http.end();
                    return false;
                }
                
                // For transient errors (negative codes = timeout/connection), retry
                http.end();
                if (httpResponseCode >= 0) {
                    return false; // Non-transient HTTP error, don't retry
                }
                // Negative code = transient (timeout, connection reset) - loop will retry
            }
        }
        
        Serial.println("All fetch attempts failed");
        return false;
    }
    
    bool fetchGlucoseHistory(GlucoseReading history[], int& count, int intervalMinutes = 1) {
        if (WiFi.status() != WL_CONNECTED || strlen(sessionId) == 0) {
            Serial.println("Can't fetch history: not connected or no session ID");
            return false;
        }
        
        HTTPClient http;
        
        // Use stack-allocated client for each request
        WiFiClientSecure client;
        client.setInsecure(); // Skip certificate validation
        
        // Dexcom readings are ~5 min apart. For the wide view we request more readings and keep
        // every Nth (below), so we still return <= MAX_GLUCOSE_HISTORY dots spaced ~intervalMinutes
        // apart. step = round(intervalMinutes / 5), clamped to >= 1.
        int step = (intervalMinutes + 2) / 5;
        if (step < 1) step = 1;
        int requestCount = MAX_GLUCOSE_HISTORY * step;
        int minutesWindow = requestCount * 5 + 60;  // a little headroom
        if (minutesWindow > 1440) minutesWindow = 1440;
        
        // Build URL
        char url[256];
        snprintf(url, sizeof(url), 
                "%s?sessionId=%s&minutes=%d&maxCount=%d", 
                glucoseUrl.c_str(), sessionId, minutesWindow, requestCount);
        
        Serial.print("History URL: ");
        Serial.println(url);
        yield(); // Allow other tasks (like touch input) to process
        
        if (!http.begin(client, url)) {
            Serial.println("Failed to set up HTTPS connection for history");
            return false;
        }
        
        http.addHeader("Content-Type", "application/json");
        http.addHeader("User-Agent", "Dexcom Share/3.0.2.11 CFNetwork/711.2.23 Darwin/14.0.0");
        http.addHeader("Accept", "application/json");
        http.setTimeout(HTTP_TIMEOUT_MS);
        
        int httpResponseCode = http.GET();
        
        if (httpResponseCode == HTTP_CODE_OK) {
            String response = http.getString();
            Serial.printf("History response length: %d\n", response.length());
            
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, response);
            
            if (!error && doc.is<JsonArray>()) {
                count = 0;
                int idx = 0;
                for (JsonObject reading : doc.as<JsonArray>()) {
                    if (count >= MAX_GLUCOSE_HISTORY) break;
                    // Keep every step-th reading (newest first) to achieve ~intervalMinutes spacing.
                    if ((idx++ % step) != 0) continue;
                    
                    history[count].value = reading["Value"].as<float>();
                    strncpy(history[count].trend, reading["Trend"].as<const char*>(), 19);
                    history[count].trend[19] = '\0';
                    
                    // Parse timestamp robustly. Dexcom returns WT as "Date(<ms>)" (sometimes
                    // "/Date(<ms>)/" and/or with a +/-HHMM tz suffix). The previous fixed-offset
                    // substring assumed the slashed "/Date(...)/" shape and silently dropped the
                    // first AND last digit of the ms value for the plain "Date(...)" form, which
                    // compressed the graph's time span ~10x. Use the same index-based extraction
                    // as the main (latest-reading) parser instead.
                    String timestampStr = reading["WT"].as<String>();
                    int openParenIndex = timestampStr.indexOf('(');
                    int closeParenIndex = timestampStr.indexOf(')');
                    if (openParenIndex != -1 && closeParenIndex != -1) {
                        String timestampOnly = timestampStr.substring(openParenIndex + 1, closeParenIndex);
                        int tzIndex = -1;
                        int dashIndex = timestampOnly.indexOf('-');
                        int plusIndex = timestampOnly.indexOf('+');
                        if (dashIndex != -1) tzIndex = dashIndex;
                        if (plusIndex != -1 && (tzIndex == -1 || plusIndex < tzIndex)) tzIndex = plusIndex;
                        if (tzIndex != -1) timestampOnly = timestampOnly.substring(0, tzIndex); // strip tz
                        timestampStr = timestampOnly;
                    }
                    
                    // Parse full timestamp for dexcomTime
                    char* endptr;
                    history[count].dexcomTime = strtoull(timestampStr.c_str(), &endptr, 10);
                    
                    // Set simple timestamp (convert to unsigned long)
                    history[count].timestamp = (unsigned long)(history[count].dexcomTime / 1000);
                    
                    count++;
                }
                
                Serial.printf("Fetched %d glucose history readings\n", count);
                http.end();
                return count > 0;
            } else {
                Serial.print("JSON parsing error in history: ");
                Serial.println(error.c_str());
            }
        } else {
            Serial.print("HTTP error when getting history: ");
            Serial.println(httpResponseCode);
            
            // Reset session credentials on 500 server errors
            if (httpResponseCode == 500) {
                Serial.println("Server error (500) detected - resetting session credentials");
                sessionId[0] = '\0';
                accountId[0] = '\0';
            }
        }
        
        http.end();
        return false;
    }
    
    const char* getSessionId() const {
        return sessionId;
    }
};

#endif // DEXCOM_CLIENT_H

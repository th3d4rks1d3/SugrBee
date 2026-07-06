#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <time.h>
#include <SPIFFS.h>
#include <FS.h>
#include <WiFiClientSecure.h>
#include <PNGdec.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <esp_wifi.h>  // Include for ESP32 WiFi power management functions
#include <Update.h>    // Include for OTA updates

// === Debug Log Buffer (must be before project includes so #define Serial captures eve
// 
// rything) ===
#define DEBUG_LOG_BUFFER_SIZE 16384  // 16KB ring buffer
char debugLogBuffer[DEBUG_LOG_BUFFER_SIZE];
int debugLogHead = 0;
int debugLogCount = 0;

// Save reference to real hardware serial before we redefine 'Serial'
HardwareSerial& _realSerial = Serial;

// DebugSerial class wraps HardwareSerial to tee all output into the debug log buffer
// Extends Stream (not just Print) so readStringUntil, available, read, etc. all work
class DebugSerial : public Stream {
public:
    void begin(unsigned long baud) { _realSerial.begin(baud); }
    size_t write(uint8_t c) override {
        debugLogBuffer[debugLogHead] = (char)c;
        debugLogHead = (debugLogHead + 1) % DEBUG_LOG_BUFFER_SIZE;
        if (debugLogCount < DEBUG_LOG_BUFFER_SIZE) debugLogCount++;
        return _realSerial.write(c);
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            debugLogBuffer[debugLogHead] = (char)buffer[i];
            debugLogHead = (debugLogHead + 1) % DEBUG_LOG_BUFFER_SIZE;
            if (debugLogCount < DEBUG_LOG_BUFFER_SIZE) debugLogCount++;
        }
        return _realSerial.write(buffer, size);
    }
    int available() override { return _realSerial.available(); }
    int read() override { return _realSerial.read(); }
    int peek() override { return _realSerial.peek(); }
    void flush() override { _realSerial.flush(); }
    void setDebugOutput(bool en) { _realSerial.setDebugOutput(en); }
    operator bool() { return (bool)_realSerial; }
};
DebugSerial debugSerial;

// Redefine Serial so ALL Serial.print/println/printf calls in this file
// and in project headers (Settings.h, DexcomClient.h) are captured
#define Serial debugSerial
// === End Debug Log Buffer Setup === 

#include "Settings.h"  // Settings class definition
#include "ArrowPNGs.h" // Embedded PNG arrow assets

// Firmware version and build timestamp - used for OTA update checks
#define FIRMWARE_VERSION "0.3.4"
#define BUILD_TIMESTAMP __DATE__ " " __TIME__  // Compile-time timestamp

// Constants for configuration - must be defined before DexcomClient.h
#define MAX_GLUCOSE_HISTORY 24   // Store last 24 readings (approx. 2 hours)
#define UPDATE_INTERVAL 300000   // 5 minutes in milliseconds
#define TOUCH_DEBOUNCE 200       // 200ms debounce period
#define RESET_PIN 0              // GPIO0 for reset button
#define MAX_IMAGE_WBYTES 240     // Width for PNG buffer
#define PNG_BUFFER_SIZE 6000     // PNG decoder buffer size

#include "DexcomClient.h"  // DexcomClient class definition
#include "LibreViewClient.h"  // LibreViewClient class definition (LibreLinkUp glucose source)

// Forward declarations of functions
void setupTime();
void pngDraw(PNGDRAW *pDraw);
bool handleTouch(unsigned long currentTime);
void updateDexcomData(unsigned long currentTime, unsigned long& lastUpdateTime, unsigned long long& lastDexcomReadingTime, bool isOverdueCheck);
void updateLibreData(unsigned long currentTime, unsigned long& lastUpdateTime, unsigned long long& lastReadingTime); // LibreView glucose fetch + display
bool fetchAutoTimezone(); // Automatic timezone detection
// Down Detector status entry - holds one service's label + status string
struct ServiceStatus {
    const char* label;
    char status[32];
};
bool fetchDexcomStatus(ServiceStatus* out, int maxCount, int& count); // Dexcom server status check
bool fetchLibreStatus(ServiceStatus* out, int maxCount, int& count);  // Libre/Abbott server status check
static bool isLibreUser(); // Detects whether credentials are configured for LibreView vs Dexcom
static unsigned long readingIntervalMs();    // Provider-aware new-reading cadence (ms): Libre 1min, Dexcom 5min
static unsigned long proactiveStartSeconds(); // When to start proactive polling after a reading (provider-aware)
static void checkServerStatusForOutage(); // Polls status page; sets g_serverOutageDetected
void checkSerialDebugCommands(); // Debug mode for manual glucose value testing

// Glucose history tracking constants and structures

// Global glucose history array (using GlucoseReading struct from DexcomClient.h)
GlucoseReading glucoseHistory[MAX_GLUCOSE_HISTORY];
int glucoseHistoryCount = 0;

// Global variables for tracking update times and states
unsigned long g_lastUpdateTime = 0;
unsigned long long g_lastDexcomReadingTime = 0;
bool g_isWaitingForNewReading = false;
unsigned long g_waitingStartTime = 0;  // Track when waiting state began
bool g_hasShownStaleMessage = false;   // Tracks if we've already forced the 'No Data' display
unsigned long g_bootTime = 0;          // Track when device booted (millis)
bool g_needsImmediateFetch = false;    // Flag to trigger immediate fetch after exiting settings
unsigned long g_lastSuccessfulFetchTime = 0;  // Track when we last got data from Dexcom API
unsigned long g_lastHistoryFetchTime = 0;     // Track when we last fetched glucose history
unsigned long g_lastNewReadingReceivedTime = 0; // Track when we actually received a NEW reading (for accurate "Just now")

// Server outage detection - polls Dexcom/Libre status pages when data has been stale for >1 hour
bool g_serverOutageDetected = false;            // True if any monitored service is reported non-operational
unsigned long g_lastOutageCheckTime = 0;        // millis() of last status page poll (0 = never checked)
bool g_debugForceOutage = false;                // Debug override: when true, keeps outage warning latched on
const unsigned long OUTAGE_STALE_THRESHOLD_SEC = 3600UL;             // 1 hour of no data before we start checking
const unsigned long OUTAGE_CHECK_INTERVAL_MS  = 15UL * 60UL * 1000UL; // Poll status page every 15 minutes
unsigned long g_lastReadingAgeAtFetchSec = 0;   // Age of reading in seconds when fetched (for time-since calc)
unsigned long g_lastNoNewReadingFetchTime = 0;  // Track successful fetches that returned same reading
unsigned long long g_nextReadingBoundaryEpochMs = 0; // Next reading boundary after last reading (provider cadence: 5min Dexcom / 1min Libre)
int g_stalePollCount = 0;                       // Count stale recovery polls since stale detected
unsigned long g_last429Time = 0;               // Track when we last hit a 429 rate limit error
int g_consecutive429Count = 0;                 // Count consecutive 429 errors for backoff
int g_consecutiveFetchFailures = 0;            // Count consecutive fetch failures for re-auth bypass
int g_proactiveCheckCount = 0;                 // Count proactive checks since waiting began
unsigned long g_libreLastAuthAttemptTime = 0;  // millis() of last LibreView re-auth attempt (0 = never)
int g_libreConsecutiveAuthFailures = 0;        // Consecutive LibreView re-auth failures (drives backoff)
float g_librePreviousGlucose = 0;              // Last LibreView reading, used to compute the reading-to-reading differential
#define CONFIG_PORTAL_TIMEOUT_SEC 90   // WiFi setup portal auto-close (1:30) if user doesn't finish
#define BOOT_GRACE_PERIOD_MS 60000     // 60 second grace period after boot before stale detection
#define FETCH_GRACE_PERIOD_MS 60000    // 60 second grace after successful fetch before showing stale
#define HISTORY_FETCH_COOLDOWN_MS 300000 // 5 minute cooldown between history fetches
#define NO_NEW_READING_COOLDOWN_MS 60000 // Cooldown after successful fetch with no new reading
#define STALE_RECOVERY_FAST_MS 10000    // First stale polls interval
#define STALE_RECOVERY_SLOW_MS 30000    // Subsequent stale polls interval
#define FRESH_READING_MAX_AGE_SEC 90     // Treat readings newer than this as "just received"

// Global variable declaration - this needs to be before any function uses it
bool shouldResetSettings = false; // Flag for settings reset

// Google Apps Script URL for receiving debug logs
// Replace this with your own deployed Apps Script web app URL
#define DEBUG_LOG_WEBHOOK_URL "https://script.google.com/macros/s/AKfycbxxFLryBBh55OgXpbCcMKrFx9Zf1W4_edYmAJ17p2pejMxFE9eIqLyKZYsCoCgfTKQcYw/exec"

// Build the debug log string from the circular buffer (oldest to newest)
String getDebugLogContents() {
    String result;
    result.reserve(debugLogCount + 200);
    
    // Add device header info
    result += "=== SugrBee Debug Log ===\n";
    result += "Firmware: ";
    result += FIRMWARE_VERSION;
    result += " (";
    result += BUILD_TIMESTAMP;
    result += ")\n";
    result += "Uptime: ";
    result += String(millis() / 1000);
    result += "s\n";
    result += "Free Heap: ";
    result += String(ESP.getFreeHeap());
    result += " bytes\n";
    if (WiFi.status() == WL_CONNECTED) {
        result += "WiFi: ";
        result += WiFi.SSID();
        result += " (";
        result += String(WiFi.RSSI());
        result += " dBm)\n";
        result += "IP: ";
        result += WiFi.localIP().toString();
        result += "\n";
    } else {
        result += "WiFi: Disconnected\n";
    }
    result += "=========================\n\n";
    
    // Read circular buffer from oldest to newest
    if (debugLogCount < DEBUG_LOG_BUFFER_SIZE) {
        // Buffer hasn't wrapped yet - data is at [0..debugLogHead)
        for (int i = 0; i < debugLogCount; i++) {
            result += debugLogBuffer[i];
        }
    } else {
        // Buffer has wrapped - oldest data starts at debugLogHead
        for (int i = 0; i < DEBUG_LOG_BUFFER_SIZE; i++) {
            result += debugLogBuffer[(debugLogHead + i) % DEBUG_LOG_BUFFER_SIZE];
        }
    }
    return result;
}

// Send the debug log to the configured webhook URL
bool sendDebugLog() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot send debug log - WiFi not connected");
        return false;
    }
    
    String logData = getDebugLogContents();
    Serial.printf("Sending debug log (%d bytes)...\n", logData.length());
    
    WiFiClientSecure client;
    client.setInsecure(); // Skip cert verification for Google Apps Script
    
    HTTPClient http;
    http.begin(client, DEBUG_LOG_WEBHOOK_URL);
    http.addHeader("Content-Type", "text/plain");
    http.setTimeout(15000); // 15 second timeout
    
    // Don't follow redirects - a 302 from Google Apps Script means it processed the request
    // Following the redirect often fails due to the complex Google redirect chain
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    
    int httpCode = http.POST(logData);
    http.end();
    
    Serial.printf("Debug log HTTP response code: %d\n", httpCode);
    
    // 302 = Google processed it and is redirecting to response page (success)
    // 200 = Direct success
    if (httpCode == 302 || httpCode == 200) {
        Serial.println("Debug log sent successfully!");
        return true;
    } else {
        Serial.printf("Debug log send failed, HTTP code: %d\n", httpCode);
        return false;
    }
}
// === End Debug Log Buffer ===

// Additional constants for configuration
#define WIFI_CHECK_INTERVAL 300000  // 5 minutes between WiFi checks
#define TIME_DISPLAY_UPDATE_INTERVAL 10000  // 10 seconds between time updates
#define MAIN_LOOP_DELAY 50       // Reduced from 100ms for better responsiveness

// Touchscreen pins - adjust based on your wiring
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

// Define colors for different BG ranges - New scheme: Low=RED, Good=GREEN, High=YELLOW
// Using brighter color constants for better visibility
#define COLOR_BG_LOW TFT_RED        // Low glucose = RED
#define COLOR_BG_NORMAL TFT_GREEN   // Normal glucose = GREEN  
#define COLOR_BG_HIGH TFT_YELLOW    // High glucose = YELLOW

// Debug mode for touch
#define DEBUG_TOUCH false  // Set to true to enable touch coordinate debugging

// Brightness control constants for ESP32-2432S028
#define LCD_BACK_LIGHT_PIN 21  // Backlight pin for ESP32-2432S028
#define LEDC_BASE_FREQ 5000
#define LEDC_TIMER_12_BIT 12
#define BRIGHTNESS_STEP 26  // 10% steps (255/10 ≈ 26)

// Helper function to set PWM value for LED brightness control (updated for newer ESP32 core)
void setBrightness(uint32_t value) {
  // Use the newer ledcWrite function directly
  // Value should be 0-255, we'll map it to 0-4095 (12-bit)
  uint32_t duty = map(value, 0, 255, 0, 4095);
  ledcWrite(LCD_BACK_LIGHT_PIN, duty);
}

// PNG file names for different trend arrows
const char* TREND_PNG_FILES[] = {
    "/none.png",             // 0 - None (no arrow)
    "/doubleup.png",         // 1 - DoubleUp
    "/up.png",               // 2 - SingleUp
    "/diagonalup.png",       // 3 - FortyFiveUp
    "/even.png",             // 4 - Flat
    "/diagonaldown.png",     // 5 - FortyFiveDown
    "/down.png",             // 6 - SingleDown
    "/doubledown.png",       // 7 - DoubleDown
    "/questionmark.png",     // 8 - NotComputable (asset name)
    "/dash.png"              // 9 - RateOutOfRange
};

// Trend direction mapping
const char *DEXCOM_TREND_DIRECTIONS[] = {
    "None",          // 0 - Unconfirmed
    "DoubleUp",      // 1 - Rapidly rising
    "SingleUp",      // 2 - Rising
    "FortyFiveUp",   // 3 - Slowly rising
    "Flat",          // 4 - Stable
    "FortyFiveDown", // 5 - Slowly falling
    "SingleDown",    // 6 - Falling
    "DoubleDown",    // 7 - Rapidly falling
    "NotComputable", // 8 - Unconfirmed
    "RateOutOfRange" // 9 - Unconfirmed
};

// PNG drawing parameters
struct PNG_DRAW_PARAMS {
    int16_t x;      // X offset on screen
    int16_t y;      // Y offset on screen
    uint16_t color; // For color tinting if needed
};

// Global instances
Settings settings;
WiFiMulti wifiMulti;
DexcomClient dexcomClient;
LibreViewClient libreClient;

// TFT Display setup
TFT_eSPI tft;
// Using setTextSize(2) to compensate for half-size rendering issue
static const GFXfont* const GLUCOSE_DISPLAY_FONT = &FreeSansBold24pt7b;
static const GFXfont* const DIFF_LARGE_FONT = &FreeSansBold18pt7b;
static const GFXfont* const DIFF_SMALL_FONT = &FreeSansBold9pt7b;

// PNG decoder
PNG png;
uint8_t pngBuffer[PNG_BUFFER_SIZE];

// Touchscreen setup
SPIClass touchscreenSPI(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// Display manager class
class DisplayManager {
private:
    // Display state variables
   bool displayNeedsFullUpdate = true;
   bool displayNeedsTimeUpdate = true;
   bool displayingInlineGraph = false; // For inline graph in arrow area
   int inlineGraphIntervalMin = 1;     // Minutes per dot: 1 = dense/native view, 15 = wide (~6h) view
   bool displayingSettings = false;
   bool displayingConfirmation = false; // For reset confirmation
   bool displayingUpdateConfirmation = false; // For "check for update?" confirmation
   bool displayingDownloadConfirmation = false; // For "download now?" confirmation after update found
   bool displayingBrightnessConfig = false; // For brightness adjustment screen
   bool showTimeDate = false; // If true, show time/date; if false, show "time since reading"
   bool displayLargeDifferential = true; // For large differential display (default ON)
   bool isDisplayInverted = false; // Track display inversion state
   bool isDisplayRotated = false; // Track display rotation state (180 degrees)
   // displayingTimeZoneConfig flag removed - automatic timezone detection now used instead
   bool updateInProgress = false; // Flag to track OTA update status
   char latestReleaseTag[20] = ""; // Store the tag name for download URL
   bool updateAvailable = false; // Flag to track if an update is available (checked at boot/midnight)
   
   // Glucose value flashing for critical values
   bool shouldFlashGlucose = false; // Whether glucose value should flash
   bool glucoseValueVisible = true; // Current visibility state of glucose value
   unsigned long lastFlashToggleTime = 0; // Last time the flash state was toggled
   const unsigned long FLASH_ON_INTERVAL = 2500; // Time glucose value stays visible (2.5 seconds)
   const unsigned long FLASH_OFF_INTERVAL = 1000; // Time glucose value stays hidden (1 second)
   static constexpr int INLINE_GRAPH_HEIGHT = 45;
   
   // Diagnostic state tracking for error messages
   static constexpr unsigned STALE_AFTER_SECONDS = 360; // 6 minutes - grace period before showing "No Data"
   static constexpr unsigned OVERDUE_AFTER_SECONDS = 300; // 5 minutes - reading is overdue (time turns red)
   enum DiagnosticState {
       DIAGNOSTIC_OK,           // Normal operation
       DIAGNOSTIC_WIFI_DISCONNECTED,    // WiFi not connected
       DIAGNOSTIC_MISSING_CREDENTIALS,  // Dexcom credentials missing
       DIAGNOSTIC_AUTH_FAILED,          // Authentication failed
       DIAGNOSTIC_LOGIN_FAILED,         // Login failed
       DIAGNOSTIC_SESSION_EXPIRED,      // Session expired
       DIAGNOSTIC_SERVER_ERROR,         // Server error (500)
       DIAGNOSTIC_CONNECTION_ERROR,     // Connection error (-1)
       DIAGNOSTIC_DATA_STALE            // Data is stale (>7 minutes)
   };
   DiagnosticState currentDiagnosticState = DIAGNOSTIC_OK;
   char diagnosticMessage[30] = ""; // Buffer for diagnostic message
   bool wasShowingDiagnostic = false; // Track if previous display was diagnostic (for layout change detection)
   
   // Glucose data
   float current_glucose_mgdl = 0;
   float previous_glucose_mgdl = 0;
   float glucose_diff = 0;
   float glucose_mmol = 0;
   char trend[20] = "Flat";
   char timestamp[10] = "N/A";
   unsigned long long lastDexcomReadingTime = 0;
   bool displayMgDL = true; // Display units flag (mg/dL or mmol/L)
   
   // Touch areas
   int glucoseTouchX = 0;
   int glucoseTouchY = 0;
   int glucoseTouchWidth = 0;
   int glucoseTouchHeight = 0;
   int arrowTouchX = 0;
   int arrowTouchY = 0;
   int arrowTouchWidth = 100;
   int arrowTouchHeight = 50;
   int titleTouchX = 0;
   int titleTouchY = 0;
   int titleTouchWidth = 0;
   int titleTouchHeight = 30;
   int settingsTouchX = 0;
   int settingsTouchY = 0;
   int settingsTouchWidth = 40;
   int settingsTouchHeight = 40;
   // Differential touch area removed - no longer toggleable
   int timeTouchX = 0;
   int timeTouchY = 0;
   int timeTouchWidth = 0;
   int timeTouchHeight = 0;
   
   // WiFi button in settings (was Reset)
   int resetButtonX = 0;
   int resetButtonY = 0;
   int resetButtonWidth = 0;
   int resetButtonHeight = 0;
   
   // Settings page navigation
   int settingsPage = 1;  // 1 = page 1, 2 = page 2
   int settingsNavDotX = 0;
   int settingsNavDotY = 0;
   int settingsNavDotSize = 20; // touch area size around the dot
   
   // Timezone button variables removed - automatic timezone detection now used
   
   int invertButtonX = 0;
   int invertButtonY = 0;
   int invertButtonWidth = 0;
   int invertButtonHeight = 0;
   
   int rotateButtonX = 0;
   int rotateButtonY = 0;
   int rotateButtonWidth = 0;
   int rotateButtonHeight = 0;
   
   // Update button for OTA updates
   int updateButtonX = 0;
   int updateButtonY = 0;
   int updateButtonWidth = 0;
   int updateButtonHeight = 0;
   
   // Brightness button
   int brightnessButtonX = 0;
   int brightnessButtonY = 0;
   int brightnessButtonWidth = 0;
   int brightnessButtonHeight = 0;
   
   // Night Mode button
   int nightModeButtonX = 0;
   int nightModeButtonY = 0;
   int nightModeButtonWidth = 0;
   int nightModeButtonHeight = 0;
   bool nightModeActive = false;
   int previousBrightness = 255; // Brightness before night mode was activated
   
   // Brightness adjustment screen buttons
   int brightnessPlusX = 0;
   int brightnessPlusY = 0;
   int brightnessPlusWidth = 0;
   int brightnessPlusHeight = 0;
   int brightnessMinusX = 0;
   int brightnessMinusY = 0;
   int brightnessMinusWidth = 0;
   int brightnessMinusHeight = 0;
   int brightnessSaveX = 0;
   int brightnessSaveY = 0;
   int brightnessSaveWidth = 0;
   int brightnessSaveHeight = 0;
   
   // Confirmation dialog buttons
   int confirmYesX = 0;
   int confirmYesY = 0;
   int confirmYesWidth = 0;
   int confirmYesHeight = 0;
   int confirmNoX = 0;
   int confirmNoY = 0;
   int confirmNoWidth = 0;
   int confirmNoHeight = 0;
   
   // Update confirmation dialog buttons
   int updateConfirmYesX = 0;
   int updateConfirmYesY = 0;
   int updateConfirmYesWidth = 0;
   int updateConfirmYesHeight = 0;
   int updateConfirmNoX = 0;
   int updateConfirmNoY = 0;
   int updateConfirmNoWidth = 0;
   int updateConfirmNoHeight = 0;
   
   // Cancel button in confirmation dialog
   int cancelButtonX = 0;
   int cancelButtonY = 0;
   int cancelButtonWidth = 0;
   int cancelButtonHeight = 0;
   
   // Time zone config screen variables removed - automatic timezone detection now used
   
   // Debug/Reset button (shared position - page-dependent)
   int debugButtonX = 0;
   int debugButtonY = 0;
   int debugButtonWidth = 0;
   int debugButtonHeight = 0;
   
   // Down Detector button (settings page 2)
   int downDetectorButtonX = 0;
   int downDetectorButtonY = 0;
   int downDetectorButtonWidth = 0;
   int downDetectorButtonHeight = 0;
   bool displayingDownDetector = false;
   
   // WiFi portal request flag
   bool shouldOpenWifiPortal = false;
   
   // Display state tracking
   char lastDisplayedTimeStr[30] = "";
   float lastDisplayedGlucose = -1;
   char lastDisplayedTrend[20] = "";
   bool lastDisplayedMgDL = true;
   
   bool initialized = false;
    
public:
    // Public alias constants for external access (no second enum declaration)
    static constexpr unsigned STALE_AFTER_SECONDS_PUBLIC = STALE_AFTER_SECONDS;              // 360s (6 minutes) grace period
    static constexpr unsigned OVERDUE_AFTER_SECONDS_PUBLIC = OVERDUE_AFTER_SECONDS;          // 300s (5 minutes) overdue threshold
    static constexpr DiagnosticState DIAGNOSTIC_OK_PUBLIC = DIAGNOSTIC_OK;                      // Normal operation
    static constexpr DiagnosticState DIAGNOSTIC_WIFI_DISCONNECTED_PUBLIC = DIAGNOSTIC_WIFI_DISCONNECTED;    // WiFi not connected
    static constexpr DiagnosticState DIAGNOSTIC_MISSING_CREDENTIALS_PUBLIC = DIAGNOSTIC_MISSING_CREDENTIALS;  // Dexcom credentials missing
    static constexpr DiagnosticState DIAGNOSTIC_AUTH_FAILED_PUBLIC = DIAGNOSTIC_AUTH_FAILED;            // Authentication failed
    static constexpr DiagnosticState DIAGNOSTIC_LOGIN_FAILED_PUBLIC = DIAGNOSTIC_LOGIN_FAILED;           // Login failed
    static constexpr DiagnosticState DIAGNOSTIC_SESSION_EXPIRED_PUBLIC = DIAGNOSTIC_SESSION_EXPIRED;     // Session expired
    static constexpr DiagnosticState DIAGNOSTIC_SERVER_ERROR_PUBLIC = DIAGNOSTIC_SERVER_ERROR;           // Server error (500)
    static constexpr DiagnosticState DIAGNOSTIC_CONNECTION_ERROR_PUBLIC = DIAGNOSTIC_CONNECTION_ERROR;   // Connection error (-1)
    static constexpr DiagnosticState DIAGNOSTIC_DATA_STALE_PUBLIC = DIAGNOSTIC_DATA_STALE;               // Data is stale (>7 minutes)
    
    // Force a full display update on next call
    void forceFullUpdate() { displayNeedsFullUpdate = true; }
    
    // Update diagnostic state based on current conditions (no drawing here)
    void updateDiagnosticState() {
        // Check WiFi connection first
        if (WiFi.status() != WL_CONNECTED) {
            currentDiagnosticState = DIAGNOSTIC_WIFI_DISCONNECTED;
            strcpy(diagnosticMessage, "Check WiFi");
            return;
        }
        
        // Check for missing credentials (need either Dexcom OR LibreView)
        bool hasDexcomCreds = strlen(settings.dexcomUsername) > 0 && strlen(settings.dexcomPassword) > 0;
        bool hasLibreCreds = strlen(settings.libreViewEmail) > 0 && strlen(settings.libreViewPassword) > 0;
        if (!hasDexcomCreds && !hasLibreCreds) {
            currentDiagnosticState = DIAGNOSTIC_MISSING_CREDENTIALS;
            strcpy(diagnosticMessage, "Check Login");
            return;
        }
        
        // Check for stale data (>= 6 minutes old) -> show "No Data"
        if (g_lastDexcomReadingTime > 0) {
            time_t now = time(nullptr);
            unsigned long long currentEpochMillis = ((unsigned long long)now) * 1000;
            unsigned long long timeSinceLastReading = currentEpochMillis - g_lastDexcomReadingTime;
            unsigned long secondsSinceLastReading = timeSinceLastReading / 1000;
            if (secondsSinceLastReading >= STALE_AFTER_SECONDS) {
                currentDiagnosticState = DIAGNOSTIC_DATA_STALE;
                strcpy(diagnosticMessage, "No Data");
                return;
            }
        }
        
        // Check for session expiry (no valid session but have Dexcom credentials)
        // Only applies to Dexcom users - LibreView handles auth differently
        if (hasDexcomCreds && !hasLibreCreds &&
            strlen(dexcomClient.getSessionId()) == 0) {
            currentDiagnosticState = DIAGNOSTIC_SESSION_EXPIRED;
            strcpy(diagnosticMessage, "Check Login");
            return;
        }
        
        // If we get here, everything seems OK
        currentDiagnosticState = DIAGNOSTIC_OK;
        strcpy(diagnosticMessage, "");
    }

    // Return current diagnostic message (or "No Data" when OK to keep consistent wording)
    const char* getDiagnosticMessage() {
        if (currentDiagnosticState == DIAGNOSTIC_OK) {
            return "No Data";
        }
        return diagnosticMessage;
    }

    // Clear diagnostic state
    void clearDiagnosticState() {
        currentDiagnosticState = DIAGNOSTIC_OK;
        diagnosticMessage[0] = '\0';
    }

    // Set diagnostic state externally
    void setDiagnosticState(DiagnosticState state) {
        currentDiagnosticState = state;
        switch (state) {
            case DIAGNOSTIC_AUTH_FAILED: strcpy(diagnosticMessage, "Auth Failed"); break;
            case DIAGNOSTIC_LOGIN_FAILED: strcpy(diagnosticMessage, "Login Failed"); break;
            case DIAGNOSTIC_SERVER_ERROR: strcpy(diagnosticMessage, "Server Error"); break;
            case DIAGNOSTIC_CONNECTION_ERROR: strcpy(diagnosticMessage, "Connection Error"); break;
            case DIAGNOSTIC_WIFI_DISCONNECTED: strcpy(diagnosticMessage, "Check WiFi"); break;
            case DIAGNOSTIC_MISSING_CREDENTIALS: strcpy(diagnosticMessage, "Check Login"); break;
            case DIAGNOSTIC_SESSION_EXPIRED: strcpy(diagnosticMessage, "Check Login"); break;
            case DIAGNOSTIC_DATA_STALE: strcpy(diagnosticMessage, "No Data"); break;
            default: diagnosticMessage[0] = '\0'; break;
        }
    }

    // Minimal time display at bottom bar with guardrails
    void updateTimeDisplay() {
        int timeSize = 2;
        int statusY = tft.height() - 30;
        char baseText[40] = "";
        char fullText[80] = "";
        char lastValueText[12] = "";
        bool isStaleReading = false;
        bool includeLastValue = false;
        uint16_t lastValueColor = TFT_WHITE;
        const char* widthText = baseText;
        time_t now = time(nullptr);
        // If time not yet synced, avoid bogus ages
        if (now < 24 * 3600) {
            strcpy(baseText, "Syncing time...");
            tft.fillRect(0, statusY - 5, tft.width(), 35, TFT_BLACK);
            int timeWidth = strlen(baseText) * 6 * timeSize;
            int timeX = (tft.width() - timeWidth) / 2;
            tft.setTextSize(timeSize);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(timeX, statusY);
            tft.print(baseText);
            displayNeedsTimeUpdate = false;
            return;
        }

        bool showingClock = showTimeDate && now >= 24 * 3600;
        if (showingClock) {
            struct tm timeinfo;
            localtime_r(&now, &timeinfo);
            char timeBuf[16];
            char dayBuf[8];
            char monthBuf[8];
            strftime(timeBuf, sizeof(timeBuf), "%I:%M %p", &timeinfo);
            if (timeBuf[0] == '0') {
                memmove(timeBuf, timeBuf + 1, strlen(timeBuf));
            }
            strftime(dayBuf, sizeof(dayBuf), "%a", &timeinfo);
            strftime(monthBuf, sizeof(monthBuf), "%b", &timeinfo);
            snprintf(baseText, sizeof(baseText), "%s %s %s %d", timeBuf, dayBuf, monthBuf, timeinfo.tm_mday);
        } else {
            // Server outage warning replaces the "X mins ago" text when an outage has been detected.
            // Flag is set by checkServerStatusForOutage() and cleared when fresh data resumes.
            if (g_serverOutageDetected) {
                if (isLibreUser()) {
                    strcpy(baseText, "Libre Server Outage");
                } else {
                    strcpy(baseText, "Dexcom Server Outage");
                }
                isStaleReading = true; // forces red coloring below
            } else if (g_lastDexcomReadingTime > 0) {
                unsigned long long currentEpochMillis = ((unsigned long long)now) * 1000ULL;
                if (currentEpochMillis <= g_lastDexcomReadingTime) {
                    // Clock is behind reading timestamp (minor NTP drift) - treat as current
                    strcpy(baseText, "Just now");
                } else {
                    unsigned long long diff = currentEpochMillis - g_lastDexcomReadingTime;
                    unsigned long secs = (unsigned long)(diff / 1000ULL);
                    
                    unsigned long mins = secs / 60;
                    isStaleReading = (secs >= STALE_AFTER_SECONDS); // Stale (red) once data is old (6 min)
                    if (secs < 60) {
                        strcpy(baseText, "Just now");
                    } else if (mins == 1) {
                        strcpy(baseText, "1 min ago");
                    } else {
                        snprintf(baseText, sizeof(baseText), "%lu mins ago", mins);
                    }
                }
            } else {
                strcpy(baseText, "No data yet");
            }
        }

        auto colorForValue = [&](float value) -> uint16_t {
            if (value >= 401) {
                return COLOR_BG_HIGH;
            }
            if (value < 40) {
                return COLOR_BG_LOW;
            }
            if (value < settings.lowThreshold) {
                return COLOR_BG_LOW;
            }
            if (value > settings.highThreshold) {
                return COLOR_BG_HIGH;
            }
            return COLOR_BG_NORMAL;
        };

        // Show last glucose value when main display would show "No Data" (data stale)
        // This ensures consistency between main display and time display
        bool shouldShowLastValue = false;
        if (!showingClock && g_lastDexcomReadingTime > 0 && current_glucose_mgdl > 0) {
            time_t now = time(nullptr);
            unsigned long long currentEpochMillis = ((unsigned long long)now) * 1000ULL;
            if (currentEpochMillis > g_lastDexcomReadingTime) {
                unsigned long long diff = currentEpochMillis - g_lastDexcomReadingTime;
                unsigned long secondsSinceLastReading = diff / 1000ULL;
                // Show last value when data is stale (same threshold as main display)
                shouldShowLastValue = (secondsSinceLastReading >= STALE_AFTER_SECONDS);
            }
        }
        
        if (shouldShowLastValue) {
            includeLastValue = true;
            snprintf(lastValueText, sizeof(lastValueText), "%.0f", current_glucose_mgdl);
            lastValueColor = colorForValue(current_glucose_mgdl);
            snprintf(fullText, sizeof(fullText), "%s (was %s)", baseText, lastValueText);
            widthText = fullText;
        } else {
            strncpy(fullText, baseText, sizeof(fullText) - 1);
            fullText[sizeof(fullText) - 1] = '\0';
            widthText = fullText;
        }

        // Skip redraw if text hasn't changed - prevents flashing
        if (strcmp(fullText, lastDisplayedTimeStr) == 0) {
            displayNeedsTimeUpdate = false;
            return;
        }
        strncpy(lastDisplayedTimeStr, fullText, sizeof(lastDisplayedTimeStr) - 1);
        lastDisplayedTimeStr[sizeof(lastDisplayedTimeStr) - 1] = '\0';

        tft.fillRect(0, statusY - 5, tft.width(), 35, TFT_BLACK);
        int timeWidth = strlen(widthText) * 6 * timeSize;
        int timeX = (tft.width() - timeWidth) / 2;
        tft.setTextSize(timeSize);
        // Time text turns red once the reading is stale (>= 6 min), matching the "No Data" point.
        bool isStaleForColor = false;
        if (!showingClock && g_lastDexcomReadingTime > 0) {
            time_t nowForColor = time(nullptr);
            unsigned long long cems = ((unsigned long long)nowForColor) * 1000ULL;
            if (cems > g_lastDexcomReadingTime) {
                unsigned long secsSince = (unsigned long)((cems - g_lastDexcomReadingTime) / 1000ULL);
                isStaleForColor = (secsSince >= STALE_AFTER_SECONDS);
            }
        }
        uint16_t baseColor = (!showingClock && (isStaleForColor || g_serverOutageDetected)) ? TFT_RED : TFT_WHITE;
        tft.setTextColor(baseColor);
        tft.setCursor(timeX, statusY);
        if (includeLastValue) {
            tft.print(baseText);
            tft.print(" (was ");
            tft.setTextColor(lastValueColor);
            tft.print(lastValueText);
            tft.setTextColor(baseColor);
            tft.print(")");
        } else {
            tft.print(baseText);
        }

        // Store touch area for time display toggling
        timeTouchX = 0;
        timeTouchY = statusY - 5;
        timeTouchWidth = tft.width();
        timeTouchHeight = 35;
        displayNeedsTimeUpdate = false;
    }

    // PNG drawing callback for inline PNG decode
    static int pngDrawCB(PNGDRAW *pDraw) {
        uint16_t lineBuffer[MAX_IMAGE_WBYTES];
        PNG_DRAW_PARAMS *params = (PNG_DRAW_PARAMS*)pDraw->pUser;
        png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
        int16_t x = params->x;
        int16_t y = params->y + pDraw->y;
        tft.pushImage(x, y, pDraw->iWidth, 1, lineBuffer);
        return 1;
    }

    // Draw a PNG centered at (x,y). Returns true on success.
    bool drawPNG(const char* filename, int16_t x, int16_t y) {
        File pngFile = SPIFFS.open(filename, "r");
        if (!pngFile) {
            return false;
        }
        size_t fileSize = pngFile.size();
        if (fileSize == 0) { pngFile.close(); return false; }
        uint8_t *buffer = (uint8_t*)malloc(fileSize);
        if (!buffer) { pngFile.close(); return false; }
        size_t bytesRead = pngFile.read(buffer, fileSize);
        pngFile.close();
        if (bytesRead != fileSize) { free(buffer); return false; }

        int16_t rc = png.openRAM(buffer, fileSize, pngDrawCB);
        if (rc != PNG_SUCCESS) { free(buffer); return false; }

        int16_t pngWidth = png.getWidth();
        int16_t pngHeight = png.getHeight();
        PNG_DRAW_PARAMS params;
        params.x = x - (pngWidth / 2);
        params.y = y - (pngHeight / 2);
        params.color = TFT_WHITE;

        rc = png.decode(&params, PNG_FAST_PALETTE);
        png.close();
        free(buffer);
        return (rc == PNG_SUCCESS);
    }

    // Draw the optional user-configured device name centered horizontally at the very top.
    // Sized to match the bottom "X mins ago" text (size 2) and vertically centered within
    // the 30px reserved top margin. Skips drawing entirely when no name is configured.
    void drawDeviceName() {
        if (strlen(settings.deviceName) == 0) return;
        
        const int nameTextSize = 2;
        const int charWidth = 6 * nameTextSize;   // default font: 6px char width * scale
        const int charHeight = 8 * nameTextSize;  // default font: 8px char height * scale
        const int topBandHeight = 30;             // reserved top margin (matches topMargin used below)
        
        int nameWidth = (int)strlen(settings.deviceName) * charWidth;
        int nameX = (tft.width() - nameWidth) / 2;
        if (nameX < 0) nameX = 0;
        int nameY = (topBandHeight - charHeight) / 2;
        if (nameY < 0) nameY = 0;
        
        // Clear just the top band to prevent leftover artifacts on re-draw,
        // then restore the hidden settings indicator dot in the top-left corner.
        tft.fillRect(0, 0, tft.width(), topBandHeight, TFT_BLACK);
        tft.fillCircle(5, 5, 2, TFT_DARKGREY);
        
        tft.setTextSize(nameTextSize);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(nameX, nameY);
        tft.print(settings.deviceName);
    }
    
    // Full trend indicator: try PNGs first, fallback to text symbols
    void drawTrendIndicator(int x, int y, const char* trendDir, uint16_t color) {
        // Map trend string to index
        int trendIndex = 0; // 0 = None
        for (int i = 0; i < 10; i++) {
            if (strcmp(trendDir, DEXCOM_TREND_DIRECTIONS[i]) == 0) {
                trendIndex = i;
                break;
            }
        }

        if (trendIndex == 0) {
            // Nothing to draw for None
            return;
        }

        // Draw PNG if asset exists and loads
        if (SPIFFS.exists(TREND_PNG_FILES[trendIndex])) {
            if (drawPNG(TREND_PNG_FILES[trendIndex], x, y)) {
                return;
            }
        }

        // Fallback to text arrow
        tft.setTextSize(3);
        tft.setTextColor(color);
        tft.setCursor(x - 15, y - 15);
        switch (trendIndex) {
            case 1: tft.print("^^"); break;  // DoubleUp
            case 2: tft.print("^"); break;   // SingleUp
            case 3: tft.print("/"); break;   // FortyFiveUp
            case 4: tft.print("->"); break;  // Flat
            case 5: tft.print("\\"); break; // FortyFiveDown
            case 6: tft.print("v"); break;   // SingleDown
            case 7: tft.print("vv"); break;  // DoubleDown
            case 8: tft.print("?"); break;   // NotComputable
            case 9: tft.print("-"); break;   // RateOutOfRange
            default: break;
        }
    }

    // Differential mode toggle removed - always use large differential display

    // Flashing related no-ops (to satisfy references)
    void checkCriticalGlucose() {}
    void checkAndToggleFlash(unsigned long /*currentTime*/) {}

    // Stubs used by flashing helpers
    void getGlucoseDisplayParams(int &x, int &y, int &w, int &h) {
        char s[10];
        if (current_glucose_mgdl >= 401) {
            strcpy(s, "HIGH");
        } else if (current_glucose_mgdl < 40) {
            strcpy(s, "LOW");
        } else {
            sprintf(s, "%d", (int)current_glucose_mgdl);
        }

        tft.setFreeFont(GLUCOSE_DISPLAY_FONT);
        w = tft.textWidth(s);
        h = tft.fontHeight();
        tft.setFreeFont(nullptr);
        tft.setTextFont(1);

        x = (tft.width() - w) / 2;
        const int topMargin = 30;
        const int spaceBetween = 65;
        const int arrowHeight = 32;
        int totalContentHeight = h + spaceBetween + arrowHeight;
        y = topMargin + ((tft.height() - totalContentHeight - topMargin - 40) / 2);
    }
    void hideGlucoseValue() { int gx,gy,gw,gh; getGlucoseDisplayParams(gx,gy,gw,gh); tft.fillRect(gx,gy,gw,gh,TFT_BLACK); }
    void showGlucoseValue() { /* minimal stub: force full redraw instead */ displayNeedsFullUpdate = true; }

    // Basic init and initial screens
    void init() {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        initialized = true;
    }
    void showInitialScreen() {
        tft.setTextSize(2);
        tft.setCursor(20, 30);
        tft.println("SugrBee Glucose Monitor");
        tft.setCursor(20, 60);
        tft.println("Starting up...");
    }
    void showConfigMode() {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        
        // Helper lambda to left-align text within a centered block
        auto printAlignedText2 = [&](const char* text, int x, int y) {
            tft.setCursor(x, y);
            tft.println(text);
        };
        
        // Title - large, centered
        tft.setTextSize(3);
        tft.setCursor((tft.width() - tft.textWidth("Setup Mode")) / 2, 10);
        tft.println("Setup Mode");
        
        // Instructions - size 2 for readability, centered as a block
        tft.setTextSize(2);
        const char* line1 = "1. Connect to WiFi:";
        const char* line2 = "SugrBee Setup";
        const char* line3 = "2. Open browser:";
        const char* line4 = "http://192.168.4.1";
        const char* line5 = "3. Enter credentials";
        const char* line6 = "Then save settings";
        int blockWidth = max(
            max(tft.textWidth(line1), tft.textWidth(line2)),
            max(max(tft.textWidth(line3), tft.textWidth(line4)), max(tft.textWidth(line5), tft.textWidth(line6)))
        );
        int blockX = (tft.width() - blockWidth) / 2;
        printAlignedText2(line1, blockX, 50);
        tft.setTextColor(TFT_CYAN);
        printAlignedText2(line2, blockX, 72);
        tft.setTextColor(TFT_WHITE);
        
        printAlignedText2(line3, blockX, 100);
        tft.setTextColor(TFT_CYAN);
        printAlignedText2(line4, blockX, 122);
        tft.setTextColor(TFT_WHITE);
        
        printAlignedText2(line5, blockX, 150);
        
        // Footer hint
        printAlignedText2(line6, blockX, 180);
        
        // Show firmware version at bottom right
        displayFirmwareVersion();
    }
    
    // Disclaimer splash screen - shown on first boot only
    void showDisclaimerScreen() {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        
        // Title - centered (text size 2 = 12px per char)
        tft.setTextSize(2);
        tft.setCursor((320 - 10 * 12) / 2, 5);  // "DISCLAIMER" = 10 chars
        tft.println("DISCLAIMER");
        
        // Disclaimer text - size 1 (6px per char), centered
        tft.setTextSize(1);
        
        // Helper lambda to center text (6px per char at size 1)
        auto centerText = [&](const char* text, int y) {
            int x = (320 - strlen(text) * 6) / 2;
            tft.setCursor(x, y);
            tft.println(text);
        };
        
        centerText("This product is NOT a medical device", 30);
        centerText("and is NOT FDA-approved. It is a display", 42);
        centerText("intended for informational and educational", 54);
        centerText("purposes only.", 66);
        
        centerText("This device should not be used for medical", 84);
        centerText("diagnosis, treatment, or decision-making.", 96);
        centerText("Always rely on official Dexcom receivers,", 108);
        centerText("mobile apps, and guidance from a qualified", 120);
        centerText("healthcare professional for managing your", 132);
        centerText("health.", 144);
        
        // Green "I Agree" button - centered
        int btnW = 160;
        int btnH = 40;
        int btnX = (320 - btnW) / 2;
        int btnY = 165;
        tft.fillRoundRect(btnX, btnY, btnW, btnH, 8, TFT_GREEN);
        tft.setTextColor(TFT_BLACK);
        tft.setTextSize(2);
        tft.setCursor(btnX + 35, btnY + 12);
        tft.println("I Agree");
        
        // Reset text color
        tft.setTextColor(TFT_WHITE);
    }
    
    // Check if touch is on the I Agree button
    bool isDisclaimerButtonPressed(int touchX, int touchY) {
        int btnX = 80;
        int btnY = 165;
        int btnW = 160;
        int btnH = 40;
        return (touchX >= btnX && touchX <= btnX + btnW &&
                touchY >= btnY && touchY <= btnY + btnH);
    }
    
    // Formats the glucose differential to match the currently selected display unit, so it's
    // never mixed with the main value (e.g. an mmol/L headline number with a raw mg/dL diff).
    // glucose_diff is always stored in mg/dL; mg/dL mode shows it as a whole number, mmol/L mode
    // converts it (divide by 18, same as the main value) and shows one decimal place.
    void formatDiffString(char* buf, size_t bufSize) const {
        if (displayMgDL) {
            if (glucose_diff > 0) {
                snprintf(buf, bufSize, "+%.0f", glucose_diff);
            } else if (glucose_diff < 0) {
                snprintf(buf, bufSize, "%.0f", glucose_diff);
            } else {
                snprintf(buf, bufSize, "+0");
            }
        } else {
            float diffMmol = glucose_diff / 18.0f;
            if (glucose_diff > 0) {
                snprintf(buf, bufSize, "+%.1f", diffMmol);
            } else if (glucose_diff < 0) {
                snprintf(buf, bufSize, "%.1f", diffMmol);
            } else {
                snprintf(buf, bufSize, "+0.0");
            }
        }
    }
    
    void updateDisplay() {
        // Only refresh if something changed
        bool glucoseChanged = (lastDisplayedGlucose != current_glucose_mgdl);
        bool trendChanged = (strcmp(lastDisplayedTrend, trend) != 0);
        bool unitsChanged = (lastDisplayedMgDL != displayMgDL);
        
        if (!glucoseChanged && !trendChanged && !unitsChanged && !displayNeedsFullUpdate) {
            // SAFEGUARD: If we're in flashing mode and value is currently hidden, but a fresh reading just arrived,
            // ensure we show the glucose immediately instead of waiting for the next flash toggle.
            if (shouldFlashGlucose && !glucoseValueVisible && g_lastDexcomReadingTime > 0) {
                time_t nowForVisibility = time(nullptr);
                unsigned long long currentEpochMillisForVisibility = ((unsigned long long)nowForVisibility) * 1000;
                unsigned long long timeSinceLastReadingForVisibility = currentEpochMillisForVisibility - g_lastDexcomReadingTime;
                unsigned long secondsSinceLastReadingForVisibility = timeSinceLastReadingForVisibility / 1000;
                // Force visibility during the first 15 seconds after a new reading
                if (secondsSinceLastReadingForVisibility < 15) {
                    glucoseValueVisible = true;
                    lastFlashToggleTime = millis(); // reset flash timer so OFF won't immediately hide it
                }
            }
            return;
        }
        
        // Check if glucose value should flash based on critical values
        // Enable flashing if glucose is at or below criticalLowGlucoseValue (if set)
        // or at or above criticalHighGlucoseValue (if set)
        shouldFlashGlucose = (settings.criticalLowGlucoseValue > 0 && 
                             (int)current_glucose_mgdl <= settings.criticalLowGlucoseValue) ||
                            (settings.criticalHighGlucoseValue > 0 && 
                             (int)current_glucose_mgdl >= settings.criticalHighGlucoseValue);
        
        // If we're starting flashing, ensure glucose is visible at first
        if (shouldFlashGlucose && glucoseChanged) {
            glucoseValueVisible = true;
            lastFlashToggleTime = millis();
        }
        
        // Save current values for next comparison
        lastDisplayedGlucose = current_glucose_mgdl;
        strncpy(lastDisplayedTrend, trend, sizeof(lastDisplayedTrend) - 1);
        lastDisplayedTrend[sizeof(lastDisplayedTrend) - 1] = '\0';
        lastDisplayedMgDL = displayMgDL;
        
        // Check if glucose data is too old (stale)
        bool isDataStale = false;
        
        // Update diagnostic state first
        updateDiagnosticState();
        
        if (g_lastDexcomReadingTime > 0) {
            time_t now = time(nullptr);
            unsigned long long currentEpochMillis = ((unsigned long long)now) * 1000;
            unsigned long long timeSinceLastReading = currentEpochMillis - g_lastDexcomReadingTime;
            unsigned long secondsSinceLastReading = timeSinceLastReading / 1000;
            
            // Once we HAVE a reading (guarded above), staleness is decided purely by its age.
            // We intentionally do NOT apply a boot grace here: a reading that is already old at
            // boot must show "No Data (was xxx)" immediately rather than dashes for 60s. The
            // clock-synced guard replaces the boot grace's real job - avoiding a bogus age from an
            // unsynced clock. Fetch grace still avoids flagging a just-received reading, but such a
            // reading is fresh anyway so it never masks a genuinely old one.
            unsigned long currentMillis = millis();
            bool clockSynced = (now >= 24 * 3600);
            // Use g_lastNewReadingReceivedTime (not g_lastSuccessfulFetchTime) so that
            // successful fetches returning the SAME old reading don't keep resetting the grace window
            bool withinFetchGrace = (g_lastNewReadingReceivedTime > 0) && 
                                    ((currentMillis - g_lastNewReadingReceivedTime) < FETCH_GRACE_PERIOD_MS);
            
            isDataStale = clockSynced && (secondsSinceLastReading >= STALE_AFTER_SECONDS) && !withinFetchGrace;
            
            // Sync diagnostic state with the grace-aware stale check
            if (!isDataStale && currentDiagnosticState == DIAGNOSTIC_DATA_STALE) {
                currentDiagnosticState = DIAGNOSTIC_OK;
                diagnosticMessage[0] = '\0';
            }
        }
        
        // Determine if we're showing diagnostic now
        bool showingDiagnosticNow = (isDataStale || currentDiagnosticState != DIAGNOSTIC_OK);
        
        // Only do full screen clear if layout is changing (normal <-> diagnostic)
        // or if explicitly requested via displayNeedsFullUpdate
        bool layoutChanged = (showingDiagnosticNow != wasShowingDiagnostic);
        if (layoutChanged || displayNeedsFullUpdate) {
            tft.fillScreen(TFT_BLACK);
        }
        wasShowingDiagnostic = showingDiagnosticNow;
        
        // Reset font state
        tft.setTextDatum(TL_DATUM);
        tft.setTextFont(0);   // Required baseline for FreeFont rendering
        tft.setTextSize(1);
        tft.setFreeFont(nullptr);
        
        // Maintain touch area for title functionality, but don't display it
        titleTouchX = 0;
        titleTouchY = 0;
        titleTouchWidth = tft.width();
        titleTouchHeight = 30;
        
        // Create hidden settings button in top left
        settingsTouchX = 0;
        settingsTouchY = 0;
        settingsTouchWidth = 40;
        settingsTouchHeight = 40;
        
        // Small indicator for settings button
        tft.fillCircle(5, 5, 2, TFT_DARKGREY);
        
        // Draw optional device name centered at top (no-op if user didn't configure one).
        // Done before the diagnostic/normal split so it appears in both layouts.
        drawDeviceName();
        
        // If data is stale or there's a diagnostic issue, display diagnostic message
        if (showingDiagnosticNow) {
            // Prepare layout parameters
            const int displayWidth = tft.width();
            const int displayHeight = tft.height();
            int glucoseSize = 8; // start large and shrink to fit if needed
            const int spaceBetween = 65;
            const int arrowHeight = 32;
            const int topMargin = 30;
            const int bottomMargin = 60;

            // Determine the message to show
            const char* messageToShow = getDiagnosticMessage();

            // Compute a fitting text size so the message fits within screen width minus margins
            tft.setTextSize(glucoseSize);
            int messageWidth = tft.textWidth(messageToShow);
            const int horizontalMargin = 20; // left/right margin for aesthetics
            const int arrowDrawNudge = 45;
            while (messageWidth > (displayWidth - horizontalMargin * 2) && glucoseSize > 2) {
                glucoseSize--;
                tft.setTextSize(glucoseSize);
                messageWidth = tft.textWidth(messageToShow);
            }

            // Calculate total content height with the chosen text size
            const int glucoseHeight = glucoseSize * 8;
            const int arrowContentHeight = arrowHeight;
            const int totalContentHeight = glucoseHeight + spaceBetween + arrowContentHeight;

            const int usableBottom = displayHeight - bottomMargin;
            int desiredArrowCenter = topMargin + ((usableBottom - topMargin) / 2);
            int minArrowCenter = topMargin + (arrowContentHeight / 2);
            int maxArrowCenter = usableBottom - (arrowContentHeight / 2);
            const int arrowVerticalNudge = 4;
            int arrowY = constrain(desiredArrowCenter, minArrowCenter, maxArrowCenter);
            int glucoseY = arrowY - (spaceBetween + (arrowContentHeight / 2)) - glucoseHeight;
            if (glucoseY < topMargin) {
                glucoseY = topMargin;
            }
            int arrowDrawY = constrain(arrowY + arrowDrawNudge, minArrowCenter, maxArrowCenter);

            // Clear the entire line area where the diagnostic text will be drawn to prevent artifacts/suffixes
            tft.fillRect(0, glucoseY, displayWidth, glucoseHeight, TFT_BLACK);

            // Draw centered diagnostic message in white, on black background
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            int prevDatum = tft.getTextDatum();
            tft.setTextDatum(MC_DATUM); // middle-center datum for easy centering
            // Center at mid-screen horizontally, and vertically center within the glucose line
            tft.drawString(messageToShow, displayWidth / 2, glucoseY + glucoseHeight / 2);
            tft.setTextDatum(prevDatum); // restore datum
            // Continue with normal display logic for trend arrow and other elements
            int arrowX = displayWidth / 2;

            // Draw trend indicator (will show question mark or appropriate symbol)
            drawTrendIndicator(arrowX, arrowDrawY, trend, TFT_WHITE);

            // Store arrow touch area
            arrowTouchX = arrowX - 50;
            arrowTouchY = arrowDrawY - 25;
            arrowTouchWidth = 100;
            arrowTouchHeight = 50;
            
            // Force time display update
            lastDisplayedTimeStr[0] = '\0';
            updateTimeDisplay();
            
            // Draw confirmation dialog if active
            if (displayingConfirmation) {
                drawConfirmationDialog();
            }
            
            // Draw update confirmation dialog if active
            if (displayingUpdateConfirmation) {
                drawUpdateConfirmationDialog();
            }
            
            return; // Exit early after displaying diagnostic message
        }
        
        // Option A: For a short period after a fresh reading, force the glucose value visible
        // This avoids the UX where time says "Just Now" but the main value is blank due to flash OFF state
        if (g_lastDexcomReadingTime > 0 && shouldFlashGlucose) {
            time_t nowForVisibility = time(nullptr);
            unsigned long long currentEpochMillisForVisibility = ((unsigned long long)nowForVisibility) * 1000;
            unsigned long long timeSinceLastReadingForVisibility = currentEpochMillisForVisibility - g_lastDexcomReadingTime;
            unsigned long secondsSinceLastReadingForVisibility = timeSinceLastReadingForVisibility / 1000;
            // Force visibility during the first 15 seconds after a new reading
            if (secondsSinceLastReadingForVisibility < 15) {
                glucoseValueVisible = true;
                lastFlashToggleTime = millis(); // reset flash timer so OFF won't immediately hide it
            }
        }
        
        // Format glucose value as string (only if data is fresh)
        char glucoseStr[10];
        int numDigits = 0;
        
        // Check if we have valid data yet (avoid showing "LOW +0" on first boot)
        bool hasValidData = (g_lastDexcomReadingTime > 0 && current_glucose_mgdl > 0);
        
        if (!hasValidData) {
            // No data received yet - show dashes
            strcpy(glucoseStr, "---");
        } else if (displayMgDL) {
            // Show mg/dL
            if (current_glucose_mgdl >= 401) {
                strcpy(glucoseStr, "HIGH");
            } else if (current_glucose_mgdl < 40) {
                strcpy(glucoseStr, "LOW");
            } else {
                sprintf(glucoseStr, "%.0f", current_glucose_mgdl);
            }
        } else {
            // Show mmol/L - convert thresholds from mg/dL to mmol/L (divide by 18)
            if (current_glucose_mgdl >= 401) { // >= 22.2 mmol/L
                strcpy(glucoseStr, "HIGH");
            } else if (current_glucose_mgdl < 40) { // < 2.2 mmol/L
                strcpy(glucoseStr, "LOW");
            } else {
                sprintf(glucoseStr, "%.1f", glucose_mmol);
            }
        }
        
        numDigits = strlen(glucoseStr);
        
        // Determine color based on glucose level using customizable thresholds
        uint16_t colorBasedOnGlucose;
        bool isRainbowValue = (current_glucose_mgdl == 100 && displayMgDL && hasValidData); // Only for 100 mg/dL
        
        if (!hasValidData) {
            // No data yet - use neutral white color
            colorBasedOnGlucose = TFT_WHITE;
        }
        // Force fixed colors for LOW and HIGH values
        else if (current_glucose_mgdl >= 401 || current_glucose_mgdl < 40) {
            // These will be displayed as HIGH or LOW text
            // Use red for HIGH and blue for LOW (non-customizable)
            if (current_glucose_mgdl >= 401) {
                colorBasedOnGlucose = COLOR_BG_HIGH; // YELLOW for HIGH
            } else {
                colorBasedOnGlucose = COLOR_BG_LOW;  // RED for LOW
            }
        }
        // For normal numeric display, use the customizable thresholds
        // CLEAN LOGIC: Same as main glucose display and inline graph
        else if (current_glucose_mgdl < settings.lowThreshold) {
            colorBasedOnGlucose = COLOR_BG_LOW;     // Below lowThreshold = RED (low)
        } else if (current_glucose_mgdl > settings.highThreshold) {
            colorBasedOnGlucose = COLOR_BG_HIGH;    // Above highThreshold = YELLOW (high)
        } else {
            colorBasedOnGlucose = COLOR_BG_NORMAL;  // Between thresholds = GREEN (good)
        }
        
        const int displayWidth = tft.width();
        const int displayHeight = tft.height();

        // Measure glucose text dimensions with 2x scaling
        tft.setTextSize(2);
        tft.setFreeFont(GLUCOSE_DISPLAY_FONT);
        int glucoseWidth = tft.textWidth(glucoseStr);
        int glucoseHeight = tft.fontHeight();
        tft.setFreeFont(nullptr);
        tft.setTextSize(1);
        tft.setTextFont(1);
        

        
        // Vertical spacing
        const int spaceBetween = 50;
        const int arrowHeight = 32;
        const int arrowContentHeight = displayingInlineGraph ? INLINE_GRAPH_HEIGHT : arrowHeight;
        const int arrowDrawNudge = 48;
        const int totalContentHeight = glucoseHeight + spaceBetween + arrowContentHeight;
        
        // No space needed at top for title since we're not displaying it
        int topMargin = 30;
        const int bottomMargin = 60; // Reserve space for footer/time area
        
        const int usableBottom = displayHeight - bottomMargin;
        int desiredArrowCenter = topMargin + ((usableBottom - topMargin) / 2);
        int minArrowCenter = topMargin + (arrowContentHeight / 2);
        int maxArrowCenter = usableBottom - (arrowContentHeight / 2);
        int arrowY = constrain(desiredArrowCenter, minArrowCenter, maxArrowCenter);
        int glucoseY = arrowY - (spaceBetween + (arrowContentHeight / 2)) - glucoseHeight;
        if (glucoseY < topMargin) {
            glucoseY = topMargin;
        }
        int startY = glucoseY;
        int arrowDrawY = constrain(arrowY + arrowDrawNudge, minArrowCenter, maxArrowCenter);
        
        // Center glucose horizontally
        int glucoseX = (displayWidth - glucoseWidth) / 2;
        
        if (displayLargeDifferential) {
            const int MIN_CENTER_GAP = 24;     // Desired gap between glucose and differential
            
            // Calculate heights and widths
            int glucoseDisplayHeight = glucoseHeight;
            
            // Format glucose and differential strings
            char diffStr[15];
            if (!hasValidData) {
                strcpy(diffStr, "");  // No diff to show when no data
            } else {
                formatDiffString(diffStr, sizeof(diffStr));
            }

            // Measure differential text width/height using FreeFont with 2x scaling
            tft.setTextSize(2);
            tft.setFreeFont(DIFF_LARGE_FONT);
            int diffPixelWidth = tft.textWidth(diffStr);
            int diffDisplayHeight = tft.fontHeight();
            tft.setFreeFont(nullptr);
            tft.setTextFont(1);
            tft.setTextSize(1);

            int glucosePixelWidth = glucoseWidth;
            
            // Always center the total width (glucose + gap + differential)
            int spacingUsed = MIN_CENTER_GAP;
            int totalWidth = glucosePixelWidth + spacingUsed + diffPixelWidth;
            
            // If too wide, reduce gap
            if (totalWidth > displayWidth) {
                spacingUsed = max(5, displayWidth - (glucosePixelWidth + diffPixelWidth));
                totalWidth = glucosePixelWidth + spacingUsed + diffPixelWidth;
            }
            
            // Center the total width on screen
            int startX = max((displayWidth - totalWidth) / 2, 0);
            int diffStartX = startX + glucosePixelWidth + spacingUsed;
            
            // Clear display area first - using max height of glucose
            tft.fillRect(0, glucoseY, displayWidth, glucoseDisplayHeight + 10, TFT_BLACK);
            
            // Determine color for differential
            uint16_t diffColor;
            if (glucose_diff > 0) {
                diffColor = TFT_WHITE;  // Positive = white
            } else if (glucose_diff < 0) {
                diffColor = TFT_RED;    // Negative = red
            } else {
                diffColor = TFT_GREEN;  // Zero = green
            }
            
            // Display large glucose value on left using FreeSansBold24pt7b smooth font
            tft.setTextSize(2);  // Force 2x scaling to compensate for half-size bug
            tft.setFreeFont(GLUCOSE_DISPLAY_FONT);
            int prevDiffDatum = tft.getTextDatum();
            tft.setTextDatum(TL_DATUM);
            
            if (!shouldFlashGlucose || glucoseValueVisible) {
                if (isRainbowValue && current_glucose_mgdl == 100) {
                    int digitOneWidth = tft.textWidth("1");
                    int digitZeroWidth = tft.textWidth("0");
                    int currentX = startX;

                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString("1", currentX, glucoseY);
                    currentX += digitOneWidth;

                    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                    tft.drawString("0", currentX, glucoseY);
                    currentX += digitZeroWidth;

                    tft.setTextColor(TFT_BLUE, TFT_BLACK);
                    tft.drawString("0", currentX, glucoseY);
                } else {
                    tft.setTextColor(colorBasedOnGlucose, TFT_BLACK);
                    tft.drawString(glucoseStr, startX, glucoseY);
                }
            }

            tft.setTextDatum(prevDiffDatum);
            tft.setFreeFont(nullptr);
            tft.setTextFont(1);
            
            int verticalOffset = (glucoseDisplayHeight - diffDisplayHeight) / 2;
            tft.setTextSize(2);  // Force 2x scaling to compensate for half-size bug
            tft.setFreeFont(DIFF_LARGE_FONT);
            int prevDiffFontDatum = tft.getTextDatum();
            tft.setTextDatum(TL_DATUM);
            tft.setTextColor(diffColor, TFT_BLACK);
            tft.drawString(diffStr, diffStartX, glucoseY + verticalOffset);
            tft.setTextDatum(prevDiffFontDatum);
            tft.setFreeFont(nullptr);
            tft.setTextFont(1);
            
            // Update touch areas
            glucoseTouchX = startX;
            glucoseTouchY = glucoseY;
            glucoseTouchWidth = glucosePixelWidth;
            glucoseTouchHeight = glucoseDisplayHeight;
        } else {
            // Draw glucose value using FreeSansBold24pt7b in normal mode
            tft.setTextSize(2);  // Force 2x scaling to compensate for half-size bug
            tft.setFreeFont(GLUCOSE_DISPLAY_FONT);
            int prevDatum = tft.getTextDatum();
            tft.setTextDatum(TL_DATUM);

            if (!shouldFlashGlucose || glucoseValueVisible) {
                if (isRainbowValue && current_glucose_mgdl == 100) {
                    int digitOneWidth = tft.textWidth("1");
                    int digitZeroWidth = tft.textWidth("0");
                    int currentX = glucoseX;

                    tft.setTextColor(TFT_RED, TFT_BLACK);
                    tft.drawString("1", currentX, glucoseY);
                    currentX += digitOneWidth;

                    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                    tft.drawString("0", currentX, glucoseY);
                    currentX += digitZeroWidth;

                    tft.setTextColor(TFT_BLUE, TFT_BLACK);
                    tft.drawString("0", currentX, glucoseY);
                } else {
                    tft.setTextColor(colorBasedOnGlucose, TFT_BLACK);
                    tft.drawString(glucoseStr, glucoseX, glucoseY);
                }
            }

            tft.setTextDatum(prevDatum);
            tft.setFreeFont(nullptr);
            tft.setTextFont(1);

            // Units display (only in normal mode)
            int unitsSize = 2;
            tft.setTextSize(unitsSize);
            tft.setTextColor(TFT_WHITE);
            
            // Position for units
            int unitsOffset = 10;
            int unitsX;
            
            if (displayMgDL) {
                unitsX = glucoseX + glucoseWidth + unitsOffset;
            } else {
                // Ensure mmol/L fits on screen
                int mmolTextWidth = 6 * unitsSize * 6;
                int maxUnitsX = displayWidth - mmolTextWidth - 5;
                unitsX = min(glucoseX + glucoseWidth + unitsOffset, maxUnitsX);
            }
            
            int unitsY = glucoseY + glucoseHeight - 36;  // Moved up 8px total
            if (unitsY > displayHeight - bottomMargin - 10) {
                unitsY = displayHeight - bottomMargin - 10;
            }
            if (unitsY < glucoseY) {
                unitsY = glucoseY;
            }
            tft.setCursor(unitsX, unitsY);
            
            // Show units only when NOT displaying NO DATA
            if (!isDataStale) {
                if (displayMgDL) {
                    tft.print("mg/dL");
                } else {
                    tft.print("mmol/L");
                }
            }
            
            // Glucose change indicator (small in normal mode) using FreeFont for consistency
            int diffX = glucoseX + glucoseWidth + 10;
            int diffY = glucoseY;
            char smallDiffStr[15];
            uint16_t diffColorSmall;
            formatDiffString(smallDiffStr, sizeof(smallDiffStr));
            if (glucose_diff > 0) {
                diffColorSmall = TFT_WHITE;
            } else if (glucose_diff < 0) {
                diffColorSmall = TFT_RED;
            } else {
                diffColorSmall = TFT_GREEN;
            }

            tft.setTextSize(2);  // Force 2x scaling to compensate for half-size bug
            tft.setFreeFont(DIFF_SMALL_FONT);
            int prevSmallDiffDatum = tft.getTextDatum();
            tft.setTextDatum(TL_DATUM);
            // Measure with 2x scaling active
            int smallDiffHeight = tft.fontHeight();
            int verticalOffsetSmall = (glucoseHeight - smallDiffHeight) / 2;
            if (verticalOffsetSmall < 0) verticalOffsetSmall = 0;
            int diffWidth = tft.textWidth(smallDiffStr);
            tft.setTextColor(diffColorSmall, TFT_BLACK);
            tft.drawString(smallDiffStr, diffX, glucoseY + verticalOffsetSmall);
            tft.setTextDatum(prevSmallDiffDatum);
            tft.setFreeFont(nullptr);
            tft.setTextFont(1);
            
            // Differential touch area removed - no longer toggleable
            
            // Update glucose touch area for normal mode
            glucoseTouchX = glucoseX;
            glucoseTouchY = glucoseY;
            glucoseTouchWidth = glucoseWidth;
            glucoseTouchHeight = glucoseHeight;
        }
        
        // Trend arrow centered horizontally
        int arrowX = displayWidth / 2;
        
        // Draw trend indicator or inline graph
        if (displayingInlineGraph && glucoseHistoryCount > 0) {
            drawInlineGraph(arrowX, arrowDrawY);
        } else {
            drawTrendIndicator(arrowX, arrowDrawY, trend, colorBasedOnGlucose);
        }
        
        // Store arrow touch area
        arrowTouchX = arrowX - 50;
        int arrowTouchHalfHeight = displayingInlineGraph ? (INLINE_GRAPH_HEIGHT / 2 + 5) : 25;
        arrowTouchY = arrowDrawY - arrowTouchHalfHeight;
        arrowTouchWidth = 100;
        arrowTouchHeight = arrowTouchHalfHeight * 2;
        
        // Debug touch areas
        if (DEBUG_TOUCH) {
            Serial.println("Touch areas defined:");
            Serial.printf("Glucose: X=%d, Y=%d, W=%d, H=%d\n", 
                         glucoseTouchX, glucoseTouchY, glucoseTouchWidth, glucoseTouchHeight);
            Serial.printf("Arrow: X=%d, Y=%d, W=%d, H=%d\n", 
                         arrowTouchX, arrowTouchY, arrowTouchWidth, arrowTouchHeight);
            Serial.printf("Title: X=%d, Y=%d, W=%d, H=%d\n", 
                         titleTouchX, titleTouchY, titleTouchWidth, titleTouchHeight);
            Serial.printf("Settings: X=%d, Y=%d, W=%d, H=%d\n", 
                         settingsTouchX, settingsTouchY, settingsTouchWidth, settingsTouchHeight);
            // Differential touch debug removed
        }
        
        // Force time display update
        lastDisplayedTimeStr[0] = '\0';
        updateTimeDisplay();
        
        // Draw confirmation dialog if active
        if (displayingConfirmation) {
            drawConfirmationDialog();
        }
        
        // Draw update confirmation dialog if active
        if (displayingUpdateConfirmation) {
            drawUpdateConfirmationDialog();
        }
        
        // No longer resetting showTimeDate flag to allow user preference to persist
        // This ensures time display stays active if user has selected it
    }
    
    void drawSettingsScreen() {
        tft.fillScreen(TFT_BLACK);
        
        // Draw "Settings" title centered
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        
        const char* titleText = "Settings (1/2)";
        int textWidth = strlen(titleText) * 12;
        int titleX = (tft.width() - textWidth) / 2;
        
        tft.setCursor(titleX, 15);
        tft.print(titleText);
        
        // Subtle line under title
        tft.drawLine(titleX, 40, titleX + textWidth, 40, TFT_DARKGREY);
        
        // Calculate grid layout (3x2 grid) for 6 buttons
        int gridCols = 3;
        int gridRows = 3; // Increased from 2 to add extra row for update button
        int gridSpacingX = 10; // Space between columns
        int gridSpacingY = 10; // Space between rows
        int gridMargin = 20;   // Margin from screen edges
        
        // Calculate button dimensions for the grid
        int buttonWidth = (tft.width() - (2 * gridMargin) - ((gridCols - 1) * gridSpacingX)) / gridCols;
        int buttonHeight = 40;  // Fixed height
        
        // Position for WiFi button (top-left position in grid)
        int buttonX = gridMargin;
        int buttonY = 60;  // Position below the title
        
        // Store button position for touch detection (reuses resetButton vars for WiFi button)
        resetButtonX = buttonX;
        resetButtonY = buttonY;
        resetButtonWidth = buttonWidth;
        resetButtonHeight = buttonHeight;
        
        // Draw WiFi button
        tft.fillRoundRect(buttonX, buttonY, buttonWidth, buttonHeight, 8, TFT_BLUE);
        tft.drawRoundRect(buttonX, buttonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // WiFi button text
        tft.setTextSize(2);
        const char* buttonText = "WiFi";  
        textWidth = strlen(buttonText) * 12;
        int buttonTextX = buttonX + (buttonWidth - textWidth) / 2;
        int buttonTextY = buttonY + (buttonHeight - 16) / 2;
        tft.setCursor(buttonTextX, buttonTextY);
        tft.print(buttonText);
        
        // Add Night Mode button (beside Reset button in top row)
        const char* nightModeText = "Night Mode";
        int nightModeTextWidth = strlen(nightModeText) * 12; // Calculate text width
        int nightModeBtnWidth = nightModeTextWidth + 20; // Add padding for better appearance
        int nightModeButtonX = buttonX + buttonWidth + gridSpacingX;
        int nightModeButtonY = buttonY;
        
        // Store night mode button position for touch detection
        this->nightModeButtonX = nightModeButtonX;
        this->nightModeButtonY = nightModeButtonY;
        this->nightModeButtonWidth = nightModeBtnWidth;
        this->nightModeButtonHeight = buttonHeight;
        
        // Draw Night Mode button - green if active, blue otherwise
        uint16_t nightModeColor = nightModeActive ? TFT_GREEN : TFT_BLUE;
        tft.fillRoundRect(nightModeButtonX, nightModeButtonY, nightModeBtnWidth, buttonHeight, 8, nightModeColor);
        tft.drawRoundRect(nightModeButtonX, nightModeButtonY, nightModeBtnWidth, buttonHeight, 8, TFT_WHITE);
        
        // Night Mode button text - black on green for visibility
        tft.setTextSize(2);
        tft.setTextColor(nightModeActive ? TFT_BLACK : TFT_WHITE);
        textWidth = nightModeTextWidth;
        int nightModeTextX = nightModeButtonX + (nightModeBtnWidth - textWidth) / 2;
        tft.setCursor(nightModeTextX, nightModeButtonY + (buttonHeight - 16) / 2);
        tft.print(nightModeText);
        tft.setTextColor(TFT_WHITE); // Reset text color
        
        // Position for Invert button (bottom-left position in grid)
        int invertButtonX = buttonX;
        int invertButtonY = buttonY + buttonHeight + gridSpacingY;
        
        // Store invert button position for touch detection
        this->invertButtonX = invertButtonX;
        this->invertButtonY = invertButtonY;
        this->invertButtonWidth = buttonWidth;
        this->invertButtonHeight = buttonHeight;
        
        // Draw Invert button
        tft.fillRoundRect(invertButtonX, invertButtonY, buttonWidth, buttonHeight, 8, TFT_BLUE);
        tft.drawRoundRect(invertButtonX, invertButtonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // Invert button text
        tft.setTextSize(2);
        const char* invertText = "Invert";
        textWidth = strlen(invertText) * 12;
        int invertTextX = invertButtonX + (buttonWidth - textWidth) / 2;
        tft.setCursor(invertTextX, invertButtonY + (buttonHeight - 16) / 2);
        tft.print(invertText);
        
        // Position for Rotate button (bottom-right position in grid)
        int rotateButtonX = invertButtonX + buttonWidth + gridSpacingX;
        int rotateButtonY = invertButtonY;
        
        // Store rotate button position for touch detection
        this->rotateButtonX = rotateButtonX;
        this->rotateButtonY = rotateButtonY;
        this->rotateButtonWidth = buttonWidth;
        this->rotateButtonHeight = buttonHeight;
        
        // Draw Rotate button
        tft.fillRoundRect(rotateButtonX, rotateButtonY, buttonWidth, buttonHeight, 8, TFT_BLUE);
        tft.drawRoundRect(rotateButtonX, rotateButtonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // Rotate button text
        tft.setTextSize(2);
        const char* rotateText = "Rotate";
        textWidth = strlen(rotateText) * 12;
        int rotateTextX = rotateButtonX + (buttonWidth - textWidth) / 2;
        tft.setCursor(rotateTextX, rotateButtonY + (buttonHeight - 16) / 2);
        tft.print(rotateText);
        
        // Add Reset button (next to Rotate on second row) - factory reset
        int debugBtnX = rotateButtonX + buttonWidth + gridSpacingX;
        int debugBtnY = rotateButtonY;
        int debugBtnWidth = buttonWidth;
        
        // Store reset/debug button position for touch detection (reuses debugButton vars)
        this->debugButtonX = debugBtnX;
        this->debugButtonY = debugBtnY;
        this->debugButtonWidth = debugBtnWidth;
        this->debugButtonHeight = buttonHeight;
        
        // Draw Reset button (red to indicate danger)
        tft.fillRoundRect(debugBtnX, debugBtnY, debugBtnWidth, buttonHeight, 8, TFT_RED);
        tft.drawRoundRect(debugBtnX, debugBtnY, debugBtnWidth, buttonHeight, 8, TFT_WHITE);
        
        // Reset button text
        tft.setTextSize(2);
        const char* resetText = "Reset";
        textWidth = strlen(resetText) * 12;
        int resetTextX = debugBtnX + (debugBtnWidth - textWidth) / 2;
        tft.setCursor(resetTextX, debugBtnY + (buttonHeight - 16) / 2);
        tft.print(resetText);
        
        // Add Update button (left side of third row)
        int updateButtonX = gridMargin; // Left side of screen
        int updateButtonY = invertButtonY + buttonHeight + gridSpacingY;
        
        // Store update button position for touch detection
        this->updateButtonX = updateButtonX;
        this->updateButtonY = updateButtonY;
        this->updateButtonWidth = buttonWidth;
        this->updateButtonHeight = buttonHeight;
        
        // Draw Update button - green if update available, blue otherwise
        uint16_t updateButtonColor = updateAvailable ? TFT_GREEN : TFT_BLUE;
        tft.fillRoundRect(updateButtonX, updateButtonY, buttonWidth, buttonHeight, 8, updateButtonColor);
        tft.drawRoundRect(updateButtonX, updateButtonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // Update button text - black text on green for visibility
        tft.setTextSize(2);
        tft.setTextColor(updateAvailable ? TFT_BLACK : TFT_WHITE);
        const char* updateText = "Update";
        textWidth = strlen(updateText) * 12;
        int updateTextX = updateButtonX + (buttonWidth - textWidth) / 2;
        tft.setCursor(updateTextX, updateButtonY + (buttonHeight - 16) / 2);
        tft.print(updateText);
        tft.setTextColor(TFT_WHITE); // Reset text color
        
        // Add Brightness button (middle of third row) - wider to fit full text
        const char* brightnessText = "Brightness";
        int brightnessTextWidth = strlen(brightnessText) * 12; // Calculate text width
        int brightnessBtnWidth = brightnessTextWidth + 20; // Add padding for better appearance
        int brightnessButtonX = updateButtonX + buttonWidth + gridSpacingX;
        int brightnessButtonY = updateButtonY;
        
        // Store brightness button position for touch detection
        this->brightnessButtonX = brightnessButtonX;
        this->brightnessButtonY = brightnessButtonY;
        this->brightnessButtonWidth = brightnessBtnWidth;
        this->brightnessButtonHeight = buttonHeight;
        
        // Draw Brightness button
        tft.fillRoundRect(brightnessButtonX, brightnessButtonY, brightnessBtnWidth, buttonHeight, 8, TFT_BLUE);
        tft.drawRoundRect(brightnessButtonX, brightnessButtonY, brightnessBtnWidth, buttonHeight, 8, TFT_WHITE);
        
        // Brightness button text
        tft.setTextSize(2);
        textWidth = brightnessTextWidth;
        int brightnessTextX = brightnessButtonX + (brightnessBtnWidth - textWidth) / 2;
        tft.setCursor(brightnessTextX, brightnessButtonY + (buttonHeight - 16) / 2);
        tft.print(brightnessText);
        
        // Explicitly clear bottom area where timer would be displayed
        tft.fillRect(0, tft.height() - 35, tft.width(), 35, TFT_BLACK);
        
        // Draw navigation dot in top-right corner (goes to page 2)
        settingsNavDotX = tft.width() - 15;
        settingsNavDotY = 15;
        tft.fillCircle(settingsNavDotX, settingsNavDotY, 4, TFT_WHITE);
        
        // Display firmware version
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        char versionText[32];
        snprintf(versionText, sizeof(versionText), "v%s", FIRMWARE_VERSION);
        int versionWidth = strlen(versionText) * 6;
        int versionX = (tft.width() - versionWidth) / 2;
        tft.setCursor(versionX, tft.height() - 25);
        tft.print(versionText);
        
        // Instructions centered and at the very bottom of the screen
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        const char* returnText = "Tap anywhere else to return";
        textWidth = strlen(returnText) * 6;  // Font width is 6 for text size 1
        int returnTextX = (tft.width() - textWidth) / 2;
        int returnTextY = tft.height() - 12;  // Very bottom positioning
        
        tft.setCursor(returnTextX, returnTextY);
        tft.print(returnText);
        
        // Draw confirmation dialog if active
        if (displayingConfirmation) {
            drawConfirmationDialog();
        }
        
        // Draw update confirmation dialog if active
        if (displayingUpdateConfirmation) {
            drawUpdateConfirmationDialog();
        }
    }
    
    // Draw settings page 2
    void drawSettingsPage2() {
        tft.fillScreen(TFT_BLACK);
        
        // Title
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        const char* titleText = "Settings (2/2)";
        int textWidth = strlen(titleText) * 12;
        int titleX = (tft.width() - textWidth) / 2;
        tft.setCursor(titleX, 15);
        tft.print(titleText);
        
        int gridMargin = 20;
        int buttonWidth = 80;
        int buttonHeight = 40;
        
        // Debug button
        int debugBtnX = gridMargin;
        int debugBtnY = 60;
        
        this->debugButtonX = debugBtnX;
        this->debugButtonY = debugBtnY;
        this->debugButtonWidth = buttonWidth;
        this->debugButtonHeight = buttonHeight;
        
        tft.fillRoundRect(debugBtnX, debugBtnY, buttonWidth, buttonHeight, 8, TFT_BLUE);
        tft.drawRoundRect(debugBtnX, debugBtnY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        const char* debugText = "Debug";
        textWidth = strlen(debugText) * 12;
        int debugTextX = debugBtnX + (buttonWidth - textWidth) / 2;
        tft.setCursor(debugTextX, debugBtnY + (buttonHeight - 16) / 2);
        tft.print(debugText);
        
        // Down Detector button (wider, below Debug)
        int ddBtnWidth = 160;
        int ddBtnX = gridMargin;
        int ddBtnY = debugBtnY + buttonHeight + 15;
        
        this->downDetectorButtonX = ddBtnX;
        this->downDetectorButtonY = ddBtnY;
        this->downDetectorButtonWidth = ddBtnWidth;
        this->downDetectorButtonHeight = buttonHeight;
        
        tft.fillRoundRect(ddBtnX, ddBtnY, ddBtnWidth, buttonHeight, 8, TFT_BLUE);
        tft.drawRoundRect(ddBtnX, ddBtnY, ddBtnWidth, buttonHeight, 8, TFT_WHITE);
        
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        const char* ddText = "Down Detector";
        int ddTextWidth = strlen(ddText) * 12;
        int ddTextX = ddBtnX + (ddBtnWidth - ddTextWidth) / 2;
        tft.setCursor(ddTextX, ddBtnY + (buttonHeight - 16) / 2);
        tft.print(ddText);
        
        // Draw navigation dot in top-left corner (goes back to page 1)
        settingsNavDotX = 15;
        settingsNavDotY = 15;
        tft.fillCircle(settingsNavDotX, settingsNavDotY, 4, TFT_WHITE);
        
        // Instructions
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        const char* returnText = "Tap anywhere else to return";
        textWidth = strlen(returnText) * 6;
        int returnTextX = (tft.width() - textWidth) / 2;
        tft.setCursor(returnTextX, tft.height() - 12);
        tft.print(returnText);
    }
    
    // Draw brightness adjustment screen
    void drawBrightnessConfigScreen() {
        tft.fillScreen(TFT_BLACK);
        
        // Draw "Brightness" title centered
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        
        const char* titleText = "Brightness";
        int textWidth = strlen(titleText) * 12;
        int titleX = (tft.width() - textWidth) / 2;
        
        tft.setCursor(titleX, 20);
        tft.print(titleText);
        
        // Subtle line under title
        tft.drawLine(titleX, 40, titleX + textWidth, 40, TFT_DARKGREY);
        
        // Show current brightness percentage
        int brightnessPercent = (settings.brightness * 100) / 255;
        char brightnessStr[20];
        snprintf(brightnessStr, sizeof(brightnessStr), "%d%%", brightnessPercent);
        
        tft.setTextSize(4);
        tft.setTextColor(TFT_CYAN);
        textWidth = strlen(brightnessStr) * 24;
        int brightnessX = (tft.width() - textWidth) / 2;
        tft.setCursor(brightnessX, 80);
        tft.print(brightnessStr);
        
        // Button dimensions
        int buttonWidth = 60;
        int buttonHeight = 50;
        int buttonSpacing = 20;
        int totalWidth = (buttonWidth * 3) + (buttonSpacing * 2);
        int startX = (tft.width() - totalWidth) / 2;
        int buttonY = 130;  // Moved up from 160 to prevent overlap with instruction text
        
        // Minus button (left, red)
        brightnessMinusX = startX;
        brightnessMinusY = buttonY;
        brightnessMinusWidth = buttonWidth;
        brightnessMinusHeight = buttonHeight;
        
        tft.fillRoundRect(brightnessMinusX, brightnessMinusY, buttonWidth, buttonHeight, 8, TFT_RED);
        tft.drawRoundRect(brightnessMinusX, brightnessMinusY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        tft.setTextSize(3);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(brightnessMinusX + 22, brightnessMinusY + 15);
        tft.print("-");
        
        // Plus button (middle, green)
        brightnessPlusX = startX + buttonWidth + buttonSpacing;
        brightnessPlusY = buttonY;
        brightnessPlusWidth = buttonWidth;
        brightnessPlusHeight = buttonHeight;
        
        tft.fillRoundRect(brightnessPlusX, brightnessPlusY, buttonWidth, buttonHeight, 8, TFT_GREEN);
        tft.drawRoundRect(brightnessPlusX, brightnessPlusY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        tft.setTextSize(3);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(brightnessPlusX + 20, brightnessPlusY + 15);
        tft.print("+");
        
        // Save button (right, blue)
        brightnessSaveX = startX + (buttonWidth + buttonSpacing) * 2;
        brightnessSaveY = buttonY;
        brightnessSaveWidth = buttonWidth;
        brightnessSaveHeight = buttonHeight;
        
        tft.fillRoundRect(brightnessSaveX, brightnessSaveY, buttonWidth, buttonHeight, 8, TFT_BLUE);
        tft.drawRoundRect(brightnessSaveX, brightnessSaveY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(brightnessSaveX + 8, brightnessSaveY + 18);
        tft.print("Save");
        
        // Instructions at bottom
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE);
        const char* instructionText = "Tap +/- to adjust, Save to apply";
        textWidth = strlen(instructionText) * 6;
        int instructionX = (tft.width() - textWidth) / 2;
        tft.setCursor(instructionX, tft.height() - 15);
        tft.print(instructionText);
    }
    
    // Show OTA update progress screen
    void showUpdateScreen(int progress) {
        // Static variables inside the method for progress tracking
        static int lastProgress = -1;
        static int lastBarWidth = 0;
        // Constants for progress bar
        const int barWidth = tft.width() - 60;
        const int barHeight = 30;
        const int barX = 30;
        const int barY = 60;
        
        // First time initialization
        if (lastProgress == -1) {
            // Clear screen and draw static elements only once
            tft.fillScreen(TFT_BLACK);
            
            // Show title
            tft.setTextSize(2);
            tft.setTextColor(TFT_WHITE);
            
            const char* titleText = "Firmware Update";
            int textWidth = strlen(titleText) * 12;
            int titleX = (tft.width() - textWidth) / 2;
            
            tft.setCursor(titleX, 20);
            tft.print(titleText);
            
            // Draw progress bar outline (only once)
            tft.drawRect(barX, barY, barWidth, barHeight, TFT_WHITE);
            
            // Show update message (static)
            tft.setTextSize(1);
            const char* updateMsg = "Downloading update from GitHub...";
            textWidth = strlen(updateMsg) * 6;
            int msgX = (tft.width() - textWidth) / 2;
            
            tft.setCursor(msgX, barY + barHeight + 50);
            tft.print(updateMsg);
            
            // Show warning about not interrupting (static)
            const char* warningMsg = "Please do not interrupt or power off";
            textWidth = strlen(warningMsg) * 6;
            int warningX = (tft.width() - textWidth) / 2;
            
            tft.setCursor(warningX, tft.height() - 30);
            tft.setTextColor(TFT_RED);
            tft.print(warningMsg);
            
            lastProgress = 0; // Initialize to 0 so first update will draw
        }
        
        // Only update if progress has changed
        if (progress != lastProgress) {
            // Calculate fill width based on progress
            int fillWidth = (progress * barWidth) / 100;
            
            // Only redraw the part of the progress bar that's changed
            if (fillWidth > lastBarWidth) {
                tft.fillRect(barX + lastBarWidth, barY, fillWidth - lastBarWidth, barHeight, TFT_GREEN);
            } else if (fillWidth < lastBarWidth) {
                // In case progress goes backward (shouldn't happen but just in case)
                tft.fillRect(barX + fillWidth, barY, lastBarWidth - fillWidth, barHeight, TFT_BLACK);
                tft.drawRect(barX, barY, barWidth, barHeight, TFT_WHITE); // Redraw border that might be erased
            }
            
            // Update progress percentage text - clear previous text first
            tft.setTextSize(2);
            char lastProgressText[10];
            sprintf(lastProgressText, "%d%%", lastProgress);
            int textWidth = strlen(lastProgressText) * 12;
            int progressX = (tft.width() - textWidth) / 2;
            tft.fillRect(progressX - 5, barY + barHeight + 20, textWidth + 10, 20, TFT_BLACK); // Clear previous text with margin
            
            // Draw new progress text
            char progressText[10];
            sprintf(progressText, "%d%%", progress);
            textWidth = strlen(progressText) * 12;
            progressX = (tft.width() - textWidth) / 2;
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(progressX, barY + barHeight + 20);
            tft.print(progressText);
            
            // Update trackers
            lastBarWidth = fillWidth;
            lastProgress = progress;
        }
    }
    
    // Compare version strings (returns: -1 if v1 < v2, 0 if equal, 1 if v1 > v2)
    int compareVersions(const char* v1, const char* v2) {
        int major1 = 0, minor1 = 0, patch1 = 0;
        int major2 = 0, minor2 = 0, patch2 = 0;
        
        sscanf(v1, "%d.%d.%d", &major1, &minor1, &patch1);
        sscanf(v2, "%d.%d.%d", &major2, &minor2, &patch2);
        
        if (major1 != major2) return (major1 > major2) ? 1 : -1;
        if (minor1 != minor2) return (minor1 > minor2) ? 1 : -1;
        if (patch1 != patch2) return (patch1 > patch2) ? 1 : -1;
        return 0;
    }
    
    // Check if a newer version is available using GitHub Releases API
    // Returns: 0 = no update, 1 = update available, -1 = error checking
    int checkForUpdate(char* remoteVersion, size_t versionBufferSize) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi not connected, cannot check for updates");
            return -1;
        }
        
        // GitHub API endpoint for latest release
        const char* apiUrl = "https://api.github.com/repos/th3d4rks1d3/SugrBee/releases/latest";
        
        Serial.println("Checking for updates via GitHub API...");
        Serial.print("Current version: ");
        Serial.println(FIRMWARE_VERSION);
        
        WiFiClientSecure client;
        client.setInsecure();
        
        HTTPClient http;
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.setTimeout(10000);
        http.begin(client, apiUrl);
        http.addHeader("User-Agent", "ESP32-SugrBee");
        http.addHeader("Accept", "application/vnd.github.v3+json");
        
        int httpCode = http.GET();
        
        if (httpCode != HTTP_CODE_OK) {
            Serial.printf("GitHub API request failed, HTTP code: %d\n", httpCode);
            http.end();
            return -1;
        }
        
        String payload = http.getString();
        http.end();
        
        // Parse JSON to get tag_name
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload);
        
        if (error) {
            Serial.print("JSON parse error: ");
            Serial.println(error.c_str());
            return -1;
        }
        
        const char* tagName = doc["tag_name"];
        if (!tagName) {
            Serial.println("No tag_name found in release");
            return -1;
        }
        
        // Store the full tag name for download URL (e.g., "v0.1.3")
        strncpy(latestReleaseTag, tagName, sizeof(latestReleaseTag) - 1);
        latestReleaseTag[sizeof(latestReleaseTag) - 1] = '\0';
        Serial.print("Release tag for download: ");
        Serial.println(latestReleaseTag);
        
        // Strip leading 'v' if present for version comparison (e.g., "v0.1.2" -> "0.1.2")
        const char* latestVersion = tagName;
        if (latestVersion[0] == 'v' || latestVersion[0] == 'V') {
            latestVersion++;
        }
        
        Serial.print("Latest release version: ");
        Serial.println(latestVersion);
        
        // Copy version to output buffer
        strncpy(remoteVersion, latestVersion, versionBufferSize - 1);
        remoteVersion[versionBufferSize - 1] = '\0';
        
        // Compare versions
        int comparison = compareVersions(latestVersion, FIRMWARE_VERSION);
        
        Serial.print("Version comparison result: ");
        Serial.println(comparison);
        
        if (comparison > 0) {
            Serial.println("Update available!");
            updateAvailable = true;
            return 1;
        } else {
            Serial.println("No update available - you have the latest version");
            updateAvailable = false;
            return 0;
        }
    }
    
    // Silent update check (no UI) - used at boot and midnight
    void checkForUpdateSilent() {
        Serial.println("Performing silent update check...");
        char remoteVersion[16];
        int result = checkForUpdate(remoteVersion, sizeof(remoteVersion));
        if (result == 1) {
            Serial.println("Silent check: Update available!");
        } else if (result == 0) {
            Serial.println("Silent check: No update available");
        } else {
            Serial.println("Silent check: Error checking for updates");
        }
    }
    
    // Getter for update availability
    bool isUpdateAvailable() const { return updateAvailable; }
    
    // Show update check result on screen (no update or error)
    void showUpdateCheckResult(int result, const char* remoteVersion) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(2);
        
        // Title
        tft.setTextColor(TFT_WHITE);
        const char* title = "Update Check";
        int titleWidth = strlen(title) * 12;
        tft.setCursor((tft.width() - titleWidth) / 2, 20);
        tft.print(title);
        
        // Current version
        tft.setTextColor(TFT_CYAN);
        tft.setCursor(10, 60);
        tft.print("Current: v");
        tft.print(FIRMWARE_VERSION);
        
        if (result == -1) {
            // Error checking
            tft.setTextColor(TFT_RED);
            tft.setCursor(10, 100);
            tft.println("Error checking");
            tft.setCursor(10, 125);
            tft.println("for updates");
        } else if (result == 0) {
            // No update available
            tft.setTextColor(TFT_GREEN);
            tft.setCursor(10, 100);
            tft.println("You have the");
            tft.setCursor(10, 125);
            tft.println("latest version!");
        }
        // Note: result == 1 (update available) is handled by drawDownloadConfirmation()
    }
    
    // Draw download confirmation dialog when update is available
    void drawDownloadConfirmation() {
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(2);
        
        // Title
        tft.setTextColor(TFT_WHITE);
        const char* title = "Update Found!";
        int titleWidth = strlen(title) * 12;
        tft.setCursor((tft.width() - titleWidth) / 2, 20);
        tft.print(title);
        
        // Current version
        tft.setTextColor(TFT_CYAN);
        tft.setCursor(10, 55);
        tft.print("Current: v");
        tft.print(FIRMWARE_VERSION);
        
        // Message
        tft.setTextColor(TFT_YELLOW);
        tft.setCursor(10, 85);
        tft.println("A newer version");
        tft.setCursor(10, 110);
        tft.println("is available!");
        
        // Draw Yes/No buttons
        int buttonWidth = 80;
        int buttonHeight = 40;
        int buttonY = 160;
        int spacing = 40;
        int totalWidth = buttonWidth * 2 + spacing;
        int startX = (tft.width() - totalWidth) / 2;
        
        // Store button positions for touch detection
        downloadYesButtonX = startX;
        downloadYesButtonY = buttonY;
        downloadYesButtonWidth = buttonWidth;
        downloadYesButtonHeight = buttonHeight;
        
        downloadNoButtonX = startX + buttonWidth + spacing;
        downloadNoButtonY = buttonY;
        downloadNoButtonWidth = buttonWidth;
        downloadNoButtonHeight = buttonHeight;
        
        // Yes button (green)
        tft.fillRoundRect(downloadYesButtonX, downloadYesButtonY, buttonWidth, buttonHeight, 8, TFT_GREEN);
        tft.drawRoundRect(downloadYesButtonX, downloadYesButtonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        tft.setTextColor(TFT_BLACK);
        const char* yesText = "Update";
        int yesTextWidth = strlen(yesText) * 12;
        tft.setCursor(downloadYesButtonX + (buttonWidth - yesTextWidth) / 2, downloadYesButtonY + (buttonHeight - 16) / 2);
        tft.print(yesText);
        
        // No button (red)
        tft.fillRoundRect(downloadNoButtonX, downloadNoButtonY, buttonWidth, buttonHeight, 8, TFT_RED);
        tft.drawRoundRect(downloadNoButtonX, downloadNoButtonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        tft.setTextColor(TFT_WHITE);
        const char* noText = "Cancel";
        int noTextWidth = strlen(noText) * 12;
        tft.setCursor(downloadNoButtonX + (buttonWidth - noTextWidth) / 2, downloadNoButtonY + (buttonHeight - 16) / 2);
        tft.print(noText);
    }
    
    // Button positions for download confirmation
    int downloadYesButtonX = 0, downloadYesButtonY = 0, downloadYesButtonWidth = 0, downloadYesButtonHeight = 0;
    int downloadNoButtonX = 0, downloadNoButtonY = 0, downloadNoButtonWidth = 0, downloadNoButtonHeight = 0;
    
    // Get download confirmation button areas
    void getDownloadConfirmationButtonAreas(int& yesX, int& yesY, int& yesW, int& yesH, 
                                            int& noX, int& noY, int& noW, int& noH) {
        yesX = downloadYesButtonX; yesY = downloadYesButtonY; 
        yesW = downloadYesButtonWidth; yesH = downloadYesButtonHeight;
        noX = downloadNoButtonX; noY = downloadNoButtonY;
        noW = downloadNoButtonWidth; noH = downloadNoButtonHeight;
    }
    
    // Getter/setter for download confirmation state
    bool isDisplayingDownloadConfirmation() const { return displayingDownloadConfirmation; }
    void setDisplayingDownloadConfirmation(bool displaying) {
        displayingDownloadConfirmation = displaying;
        displayNeedsFullUpdate = true;
    }
    
    // Perform OTA update from GitHub
    bool performOtaUpdate() {
        if (WiFi.status() != WL_CONNECTED) {
            // Show error message
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_RED);
            tft.setCursor(20, 60);
            tft.println("WiFi not connected!");
            tft.setCursor(20, 90);
            tft.println("Update failed");
            delay(3000);
            updateInProgress = false;
            return false;
        }
        
        updateInProgress = true;
        
        // Show initial update screen
        showUpdateScreen(0);
        
        // Build GitHub release URL using the stored tag name
        char firmwareUrl[150];
        snprintf(firmwareUrl, sizeof(firmwareUrl), 
                 "https://github.com/th3d4rks1d3/SugrBee/releases/download/%s/SugrBee-Update.bin",
                 latestReleaseTag);
        
        Serial.println("Starting firmware download from: ");
        Serial.println(firmwareUrl);
        
        WiFiClientSecure client;
        client.setInsecure(); // Skip certificate verification
        
        HTTPClient http;
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // Follow redirects
        http.setTimeout(20000); // Increase timeout to 20 seconds
        http.begin(client, firmwareUrl);
        http.addHeader("User-Agent", "ESP32");
        
        // Get file size
        Serial.println("Sending HTTP GET request...");
        int httpCode = http.GET();
        Serial.print("HTTP response code: ");
        Serial.println(httpCode);
        
        if (httpCode != HTTP_CODE_OK) {
            Serial.printf("Firmware download failed, error: %d\n", httpCode);
            // Show error message
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_RED);
            tft.setCursor(20, 60);
            tft.printf("Download error: %d", httpCode);
            tft.setCursor(20, 90);
            tft.println("Update failed");
            delay(3000);
            http.end();
            updateInProgress = false;
            return false;
        }
        
        int contentLength = http.getSize();
        Serial.print("Content length: ");
        Serial.println(contentLength);
        
        if (contentLength <= 0) {
            Serial.println("Error: Content-Length not defined");
            // Show error message
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_RED);
            tft.setCursor(20, 60);
            tft.println("Invalid content size");
            tft.setCursor(20, 90);
            tft.println("Update failed");
            delay(3000);
            http.end();
            updateInProgress = false;
            return false;
        }
        
        // Begin OTA update
        if (!Update.begin(contentLength)) {
            Serial.println("Not enough space for update");
            http.end();
            updateInProgress = false;
            return false;
        }
        
        // Create buffer for downloading firmware
        uint8_t buffer[1024];
        WiFiClient* stream = http.getStreamPtr();
        
        // Download and write firmware
        int totalBytes = 0;
        while (http.connected() && (totalBytes < contentLength)) {
            // Read data from HTTP stream
            size_t bytesAvailable = stream->available();
            if (bytesAvailable) {
                size_t bytesToRead = min((size_t)1024, bytesAvailable);
                size_t bytesRead = stream->readBytes(buffer, bytesToRead);
                
                // Write to Update object
                if (Update.write(buffer, bytesRead) != bytesRead) {
                    Serial.println("Error writing update");
                    Update.abort();
                    http.end();
                    updateInProgress = false;
                    return false;
                }
                
                totalBytes += bytesRead;
                
                // Update progress on screen
                int progress = (totalBytes * 100) / contentLength;
                showUpdateScreen(progress);
            }
            delay(1); // Small delay to prevent watchdog reset
        }
        
        http.end();
        
        // Finalize update
        if (Update.end(true)) {
            Serial.println("Update complete, rebooting...");
            
            // Show completion message
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_GREEN);
            tft.setCursor(20, 60);
            tft.println("Update complete!");
            tft.setCursor(20, 90);
            tft.println("Rebooting...");
            
            delay(2000); // Give user time to read the message
            ESP.restart();
            return true;
        } else {
            Serial.printf("Update error: %s\n", Update.errorString());
            updateInProgress = false;
            return false;
        }
    }
    
    // drawTimeZoneConfigScreen() function removed - automatic timezone detection now used instead
    
    // IMPROVED TOGGLE METHODS
    
    void toggleDisplayMode() {
        displayMgDL = !displayMgDL;
        
        // Always ensure glucose is visible when toggling display mode
        // This prevents disappearing glucose when flashing is active
        glucoseValueVisible = true;
        lastFlashToggleTime = millis(); // Reset flash timer
        
        displayNeedsFullUpdate = true;
    }
    
    void toggleSettings(bool show) {
        // Only update state if it's changing
        if (displayingSettings != show) {
            displayingSettings = show;
            
            if (show) {
                // Make sure other views are closed
                displayingConfirmation = false;
                settingsPage = 1; // Always start on page 1
                
                // Force a full screen refresh
                tft.fillScreen(TFT_BLACK);
                displayNeedsFullUpdate = true;
            } else {
                // Going back to main screen
                settingsPage = 1; // Reset page for next time
                tft.fillScreen(TFT_BLACK);
                displayLargeDifferential = true; // Always revert to large view after settings
                displayNeedsFullUpdate = true;
                
                // Trigger immediate glucose fetch when returning to main screen
                g_needsImmediateFetch = true;
                
                // Reset polling state so the immediate fetch + recovery cycle starts fresh.
                // Without this, stale flags from before entering settings (e.g. an in-flight
                // "waiting for new reading" cycle) can block the proactive-check path,
                // causing the screen to stay frozen on "X mins ago" after long settings sessions.
                g_isWaitingForNewReading = false;
                g_proactiveCheckCount = 0;
                g_consecutive429Count = 0;       // Drop any rate-limit backoff
                g_consecutiveFetchFailures = 0;  // Drop fetch-failure latch
                g_lastUpdateTime = 0;            // Allow the next staged poll to fire immediately if needed
                g_nextReadingBoundaryEpochMs = 0;
            }
        }
    }
    
    void toggleConfirmation(bool show) {
        // Only update state if it's changing
        if (displayingConfirmation != show) {
            displayingConfirmation = show;
            
            // Force a full screen refresh if showing confirmation
            if (show) {
                displayNeedsFullUpdate = true;
            }
        }
    }
    
    // Time display toggle methods
    void toggleTimeDisplay() { 
        showTimeDate = !showTimeDate; 
        displayNeedsTimeUpdate = true; 
        
        // Debug logging to track time display mode changes
        Serial.print("Time display toggled to: ");
        Serial.println(showTimeDate ? "Clock/Date" : "Minutes since reading");
    }
    bool isShowingTimeDate() const { return showTimeDate; }
    
    // Public method to check if update confirmation is active
    bool isDisplayingUpdateConfirmation() const { 
        return displayingUpdateConfirmation; 
    }
    
    // All manual timezone functions removed - now using automatic timezone detection
    
    
    // Process reset credentials action
    bool handleResetCredentialsTouch(int x, int y) {
        // Check if confirmation is active
        if (displayingConfirmation) {
            // Check if "Yes" button was pressed
            if (x >= confirmYesX && x <= (confirmYesX + confirmYesWidth) && 
                y >= confirmYesY && y <= (confirmYesY + confirmYesHeight)) {
                
                // User confirmed reset
                displayingConfirmation = false;
                displayingSettings = false;
                
                // Mark global flag to trigger reset on next restart
                shouldResetSettings = true;
                
                // Show reset message
                tft.fillScreen(TFT_BLACK);
                tft.setTextSize(2);
                tft.setCursor(20, 60);
                tft.println("Resetting settings...");
                
                // Reset WiFi settings BEFORE restarting
                WiFiManager wifiManager;
                wifiManager.resetSettings();
                
                // Reset Dexcom credentials
                settings.reset();
                
                tft.setCursor(20, 90);
                tft.println("Restarting...");
                tft.setCursor(20, 120);
                tft.println("Connect to WiFi network:");
                tft.setCursor(20, 150);
                tft.println("SugrBee Setup");
                
                delay(3000);
                return true;  // Return true to indicate we should restart
            }
            
            // Check if "No" button was pressed
            if (x >= confirmNoX && x <= (confirmNoX + confirmNoWidth) && 
                y >= confirmNoY && y <= (confirmNoY + confirmNoHeight)) {
                
                // User cancelled - hide dialog but stay in settings
                displayingConfirmation = false;
                displayNeedsFullUpdate = true;
                return false;  // Don't restart
            }
            
            return false;  // Didn't press any relevant button
        }
        // Check if settings is active and WiFi button was pressed (formerly Reset position)
        if (displayingSettings && settingsPage == 1 &&
            x >= resetButtonX && x <= (resetButtonX + resetButtonWidth) && 
            y >= resetButtonY && y <= (resetButtonY + resetButtonHeight)) {
            
            // WiFi button tapped - will be handled externally via shouldOpenWifiPortal flag
            shouldOpenWifiPortal = true;
        }
        
        // Check if settings is active and invert button was pressed (page 1 only)
        if (displayingSettings && settingsPage == 1 && 
            x >= invertButtonX && x <= (invertButtonX + invertButtonWidth) && 
            y >= invertButtonY && y <= (invertButtonY + invertButtonHeight)) {
            
            // Toggle display inversion
            isDisplayInverted = !isDisplayInverted;
            tft.invertDisplay(isDisplayInverted);
            displayNeedsFullUpdate = true;
            settings.isDisplayInverted = isDisplayInverted;
            settings.save();
        }
        
        // Check if settings is active and rotate button was pressed (page 1 only)
        if (displayingSettings && settingsPage == 1 && 
            x >= rotateButtonX && x <= (rotateButtonX + rotateButtonWidth) && 
            y >= rotateButtonY && y <= (rotateButtonY + rotateButtonHeight)) {
            
            // Toggle display rotation
            isDisplayRotated = !isDisplayRotated;
            
            if (isDisplayRotated) {
                tft.setRotation(3); // 180 degree rotation
                touchscreen.setRotation(3); // Match rotation on touchscreen
            } else {
                tft.setRotation(1); // Normal rotation
                touchscreen.setRotation(1); // Match rotation on touchscreen
            }
            
            displayNeedsFullUpdate = true;
            settings.isDisplayRotated = isDisplayRotated;
            settings.save();
        }
        // Update button handling is centralized in TouchHandler settings branch
        
        // Check if settings is active and brightness button was pressed (page 1 only)
        if (displayingSettings && settingsPage == 1 && 
            x >= brightnessButtonX && x <= (brightnessButtonX + brightnessButtonWidth) && 
            y >= brightnessButtonY && y <= (brightnessButtonY + brightnessButtonHeight)) {
            
            // Show brightness adjustment screen
            displayingBrightnessConfig = true;
            displayingSettings = false;
            displayNeedsFullUpdate = true;
            return false; // Don't fall through to brightness touch handler with stale coordinates
        }
        
        // Handle brightness adjustment screen touches
        // GUARD: Only process if brightness was already showing BEFORE this function was called
        if (displayingBrightnessConfig && !displayingSettings) {
            // Check if minus button was pressed
            if (x >= brightnessMinusX && x <= (brightnessMinusX + brightnessMinusWidth) && 
                y >= brightnessMinusY && y <= (brightnessMinusY + brightnessMinusHeight)) {
                
                // Decrease brightness by 10% using clean percentage steps
                // Use proper rounding to get current percentage
                int currentPercent = (settings.brightness * 100 + 127) / 255; // Add 127 for proper rounding
                
                // Round to nearest 10% to ensure clean steps
                currentPercent = ((currentPercent + 5) / 10) * 10;
                
                int newPercent = currentPercent - 10;
                if (newPercent < 10) newPercent = 10; // Minimum 10%
                
                // Convert back with proper rounding
                int newBrightness = (newPercent * 255 + 50) / 100;
                settings.brightness = newBrightness;
                
                // Apply brightness immediately for preview
                setBrightness(settings.brightness);
                
                // Redraw brightness screen to show new value
                displayNeedsFullUpdate = true;
                return false;
            }
            
            // Check if plus button was pressed
            if (x >= brightnessPlusX && x <= (brightnessPlusX + brightnessPlusWidth) && 
                y >= brightnessPlusY && y <= (brightnessPlusY + brightnessPlusHeight)) {
                
                // Increase brightness by 10% using clean percentage steps
                // Use proper rounding to get current percentage
                int currentPercent = (settings.brightness * 100 + 127) / 255; // Add 127 for proper rounding
                
                // Round to nearest 10% to ensure clean steps
                currentPercent = ((currentPercent + 5) / 10) * 10;
                
                int newPercent = currentPercent + 10;
                if (newPercent > 100) newPercent = 100; // Maximum 100%
                
                // Convert back with proper rounding
                int newBrightness = (newPercent * 255 + 50) / 100;
                settings.brightness = newBrightness;
                
                // Apply brightness immediately for preview
                setBrightness(settings.brightness);
                
                // Redraw brightness screen to show new value
                displayNeedsFullUpdate = true;
                return false;
            }
            
            // Check if save button was pressed
            if (x >= brightnessSaveX && x <= (brightnessSaveX + brightnessSaveWidth) && 
                y >= brightnessSaveY && y <= (brightnessSaveY + brightnessSaveHeight)) {
                
                // Save brightness setting
                settings.save();
                
                // Return to settings screen
                displayingBrightnessConfig = false;
                displayingSettings = true;
                displayNeedsFullUpdate = true;
                return false;
            }
            
            // Check if user tapped outside buttons to return
            // Don't return to settings if they tapped on a button area
            bool tappedOnButton = (x >= brightnessMinusX && x <= (brightnessMinusX + brightnessMinusWidth) && 
                                  y >= brightnessMinusY && y <= (brightnessMinusY + brightnessMinusHeight)) ||
                                 (x >= brightnessPlusX && x <= (brightnessPlusX + brightnessPlusWidth) && 
                                  y >= brightnessPlusY && y <= (brightnessPlusY + brightnessPlusHeight)) ||
                                 (x >= brightnessSaveX && x <= (brightnessSaveX + brightnessSaveWidth) && 
                                  y >= brightnessSaveY && y <= (brightnessSaveY + brightnessSaveHeight));
            
            if (!tappedOnButton) {
                // User tapped outside buttons - return to settings without saving
                displayingBrightnessConfig = false;
                displayingSettings = true;
                displayNeedsFullUpdate = true;
                return false;
            }
            
            // IMPORTANT: Return false here to prevent processing other settings buttons
            // when we're on the brightness screen
            return false;
        }
        
        // Update confirmation handling is centralized in TouchHandler's early modal section.
        // No direct OTA triggering here.
        
        return false;
    }
    
    // Set glucose data
    void setGlucoseData(float current, float previous, const char* newTrend, const char* newTimestamp, unsigned long long dexcomTime) {
        // Skip redundant updates - if timestamp is the same, no need to redraw
        if (dexcomTime == lastDexcomReadingTime && current == current_glucose_mgdl) {
            Serial.println("Skipping redundant display update - same reading");
            return;
        }
        
        // Add guard rails for invalid values
        // Values <= 0 should be shown as LOW
        if (current <= 0) {
            Serial.println("Warning: Received invalid glucose value <= 0, treating as LOW");
        }
        
        // Always update glucose values so display logic stays in sync
        current_glucose_mgdl = current;
        previous_glucose_mgdl = previous;
        glucose_diff = current - previous;
        glucose_mmol = current / 18.0;
        
        // Update trend and timestamp strings
        strncpy(trend, newTrend, sizeof(trend) - 1);
        trend[sizeof(trend) - 1] = '\0';
        strncpy(timestamp, newTimestamp, sizeof(timestamp) - 1);
        timestamp[sizeof(timestamp) - 1] = '\0';
        
        // Store the reading time for "time ago" display
        lastDexcomReadingTime = dexcomTime;
        
        // No longer forcing "time since reading" view when new readings arrive
        // This allows user's preferred time display setting to persist
        
        // Check if this is a critical glucose value that should flash
        checkCriticalGlucose();
        
        // Mark display for update
        displayNeedsFullUpdate = true;
        displayNeedsTimeUpdate = true;
    }
    
    // Touch handling helpers
    void getTouchAreas(int& gX, int& gY, int& gW, int& gH, 
                      int& aX, int& aY, int& aW, int& aH,
                      int& tX, int& tY, int& tW, int& tH,
                      int& sX, int& sY, int& sW, int& sH) {
        gX = glucoseTouchX; gY = glucoseTouchY; gW = glucoseTouchWidth; gH = glucoseTouchHeight;
        aX = arrowTouchX; aY = arrowTouchY; aW = arrowTouchWidth; aH = arrowTouchHeight;
        tX = titleTouchX; tY = titleTouchY; tW = titleTouchWidth; tH = titleTouchHeight;
        sX = settingsTouchX; sY = settingsTouchY; sW = settingsTouchWidth; sH = settingsTouchHeight;
    }

    void getTimeTouchArea(int& tx, int& ty, int& tw, int& th) {
        tx = timeTouchX; ty = timeTouchY; tw = timeTouchWidth; th = timeTouchHeight;
    }
    
    // Get Reset button position
    void getResetButtonArea(int& rX, int& rY, int& rW, int& rH) {
        rX = resetButtonX; rY = resetButtonY; rW = resetButtonWidth; rH = resetButtonHeight;
    }
    
    // Get confirmation button positions
    void getConfirmButtonAreas(int& yesX, int& yesY, int& yesW, int& yesH,
                             int& noX, int& noY, int& noW, int& noH) {
        yesX = confirmYesX; yesY = confirmYesY; yesW = confirmYesWidth; yesH = confirmYesHeight;
        noX = confirmNoX; noY = confirmNoY; noW = confirmNoWidth; noH = confirmNoHeight;
    }
    
    // getTimeZoneButtonArea function removed - automatic timezone detection now used instead
    
    // Get invert button position
    void getInvertButtonArea(int& iX, int& iY, int& iW, int& iH) {
        iX = invertButtonX; iY = invertButtonY; iW = invertButtonWidth; iH = invertButtonHeight;
    }
    
    // Get rotate button position
    void getRotateButtonArea(int& rX, int& rY, int& rW, int& rH) {
        rX = rotateButtonX; rY = rotateButtonY; rW = rotateButtonWidth; rH = rotateButtonHeight;
    }
    
    // Get update button position
    void getUpdateButtonArea(int& uX, int& uY, int& uW, int& uH) {
        uX = updateButtonX; uY = updateButtonY; uW = updateButtonWidth; uH = updateButtonHeight;
    }
    
    // Get cancel button position
    void getCancelButtonArea(int& cX, int& cY, int& cW, int& cH) {
        cX = cancelButtonX; cY = cancelButtonY; cW = cancelButtonWidth; cH = cancelButtonHeight;
    }
    
    // State getters
    bool needsFullUpdate() const { return displayNeedsFullUpdate; }
    bool needsTimeUpdate() const { return displayNeedsTimeUpdate; }
    void clearFullUpdate() { displayNeedsFullUpdate = false; }
    void clearTimeUpdate() { displayNeedsTimeUpdate = false; }
    bool isDisplayingSettings() const { return displayingSettings; }
    bool isDisplayingConfirmation() const {
       return displayingConfirmation;
   }
   
   // The isDisplayingUpdateConfirmation method is already defined at line ~2607
   
   // Set flag for displaying update confirmation dialog
   void setDisplayingUpdateConfirmation(bool displaying) {
       displayingUpdateConfirmation = displaying;
       displayNeedsFullUpdate = true;
   }
   
   // Get the coordinates of update confirmation dialog buttons
   void getUpdateConfirmationButtonAreas(int &yesX, int &yesY, int &yesW, int &yesH, 
                                       int &noX, int &noY, int &noW, int &noH) {
       yesX = updateConfirmYesX;
       yesY = updateConfirmYesY;
       yesW = updateConfirmYesWidth;
       yesH = updateConfirmYesHeight;
       noX = updateConfirmNoX;
       noY = updateConfirmNoY;
       noW = updateConfirmNoWidth;
       noH = updateConfirmNoHeight;
   }
   
    bool isDisplayingLargeDifferential() const { return displayLargeDifferential; }
    
    // Rotation state setter (for applying saved settings on startup)
    void setRotationState(bool rotated) { isDisplayRotated = rotated; }
    
    // Get current glucose value accessor
    float getCurrentGlucose() const { return current_glucose_mgdl; }
    
    // Add glucose reading to history
    void addGlucoseToHistory(const char* trendSymbol) {
        // Shift the array to make space for new reading
        for (int i = MAX_GLUCOSE_HISTORY - 1; i > 0; i--) {
            glucoseHistory[i] = glucoseHistory[i-1];
        }
        
        // Add the current reading to position 0
        glucoseHistory[0].value = current_glucose_mgdl;
        strcpy(glucoseHistory[0].trend, "");
        glucoseHistory[0].dexcomTime = millis();
        glucoseHistory[0].timestamp = millis(); // Use current millis as timestamp
        
        // Update count
        if (glucoseHistoryCount < MAX_GLUCOSE_HISTORY) {
            glucoseHistoryCount++;
        }
    }
    
    void drawConfirmationDialog() {
        // Dialog dimensions
        int dialogWidth = 240;
        int dialogHeight = 180;  // Reduced height since we have fewer options now
        int dialogX = (tft.width() - dialogWidth) / 2;
        int dialogY = (tft.height() - dialogHeight) / 2;
        
        // Draw dialog background with shadow effect
        tft.fillRoundRect(dialogX + 3, dialogY + 3, dialogWidth, dialogHeight, 8, TFT_DARKGREY);
        tft.fillRoundRect(dialogX, dialogY, dialogWidth, dialogHeight, 8, TFT_NAVY);
        tft.drawRoundRect(dialogX, dialogY, dialogWidth, dialogHeight, 8, TFT_WHITE);
        
        // Dialog title
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        
        const char* titleText = "Reset Device";
        int textWidth = strlen(titleText) * 12;
        int titleX = dialogX + (dialogWidth - textWidth) / 2;
        
        tft.setCursor(titleX, dialogY + 15);
        tft.print(titleText);
        
        // Dialog message
        tft.setTextSize(1);
        
        const char* messageText1 = "This will reset all credentials";
        const char* messageText2 = "including WiFi and Dexcom/LibreView";
        const char* messageText3 = "You will need to re-enter them.";
        
        textWidth = strlen(messageText1) * 6;
        int messageX = dialogX + (dialogWidth - textWidth) / 2;
        tft.setCursor(messageX, dialogY + 45);
        tft.print(messageText1);
        
        textWidth = strlen(messageText2) * 6;
        messageX = dialogX + (dialogWidth - textWidth) / 2;
        tft.setCursor(messageX, dialogY + 60);
        tft.print(messageText2);
        
        textWidth = strlen(messageText3) * 6;
        messageX = dialogX + (dialogWidth - textWidth) / 2;
        tft.setCursor(messageX, dialogY + 80);
        tft.print(messageText3);
        
        // Yes button (left)
        int buttonWidth = 80;
        int buttonHeight = 40;
        int buttonSpacing = 40;
        int yesButtonX = dialogX + (dialogWidth / 2) - buttonWidth - (buttonSpacing / 2);
        int buttonY = dialogY + dialogHeight - buttonHeight - 25;  // Position bottom action buttons
        
        confirmYesX = yesButtonX;
        confirmYesY = buttonY;
        confirmYesWidth = buttonWidth;
        confirmYesHeight = buttonHeight;
        
        tft.fillRoundRect(yesButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_GREEN);
        tft.drawRoundRect(yesButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // Yes button text
        tft.setTextSize(2);
        const char* yesText = "Yes";
        textWidth = strlen(yesText) * 12;
        int yesTextX = yesButtonX + (buttonWidth - textWidth) / 2;
        int buttonTextY = buttonY + (buttonHeight - 16) / 2;
        tft.setCursor(yesTextX, buttonTextY);
        tft.print(yesText);
        
        // No button (right)
        int noButtonX = dialogX + (dialogWidth / 2) + (buttonSpacing / 2);
        
        confirmNoX = noButtonX;
        confirmNoY = buttonY;
        confirmNoWidth = buttonWidth;
        confirmNoHeight = buttonHeight;
        
        tft.fillRoundRect(noButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_RED);
        tft.drawRoundRect(noButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // No button text
        const char* noText = "No";
        textWidth = strlen(noText) * 12;
        int noTextX = noButtonX + (buttonWidth - textWidth) / 2;
        tft.setCursor(noTextX, buttonTextY);
        tft.print(noText);
    }
    
    void drawUpdateConfirmationDialog() {
        // Dialog dimensions
        int dialogWidth = 240;
        int dialogHeight = 180;
        int dialogX = (tft.width() - dialogWidth) / 2;
        int dialogY = (tft.height() - dialogHeight) / 2;
        
        // Draw dialog background with shadow effect
        tft.fillRoundRect(dialogX + 3, dialogY + 3, dialogWidth, dialogHeight, 8, TFT_DARKGREY);
        tft.fillRoundRect(dialogX, dialogY, dialogWidth, dialogHeight, 8, TFT_NAVY);
        tft.drawRoundRect(dialogX, dialogY, dialogWidth, dialogHeight, 8, TFT_WHITE);
        
        // Dialog title
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        
        const char* titleText = "Firmware Update";
        int textWidth = strlen(titleText) * 12;
        int titleX = dialogX + (dialogWidth - textWidth) / 2;
        
        tft.setCursor(titleX, dialogY + 15);
        tft.print(titleText);
        
        // Dialog message
        tft.setTextSize(1);
        
        const char* messageText1 = "Download and install the latest";
        const char* messageText2 = "firmware update?";
        const char* messageText3 = "Device will restart when complete.";
        
        textWidth = strlen(messageText1) * 6;
        int messageX = dialogX + (dialogWidth - textWidth) / 2;
        tft.setCursor(messageX, dialogY + 45);
        tft.print(messageText1);
        
        textWidth = strlen(messageText2) * 6;
        messageX = dialogX + (dialogWidth - textWidth) / 2;
        tft.setCursor(messageX, dialogY + 60);
        tft.print(messageText2);
        
        textWidth = strlen(messageText3) * 6;
        messageX = dialogX + (dialogWidth - textWidth) / 2;
        tft.setCursor(messageX, dialogY + 80);
        tft.print(messageText3);
        
        // Yes button (left)
        int buttonWidth = 80;
        int buttonHeight = 40;
        int buttonSpacing = 40;
        int yesButtonX = dialogX + (dialogWidth / 2) - buttonWidth - (buttonSpacing / 2);
        int buttonY = dialogY + dialogHeight - buttonHeight - 25;  // Position bottom action buttons
        
        updateConfirmYesX = yesButtonX;
        updateConfirmYesY = buttonY;
        updateConfirmYesWidth = buttonWidth;
        updateConfirmYesHeight = buttonHeight;
        
        tft.fillRoundRect(yesButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_GREEN);
        tft.drawRoundRect(yesButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // Yes button text
        tft.setTextSize(2);
        const char* yesText = "Yes";
        textWidth = strlen(yesText) * 12;
        int yesTextX = yesButtonX + (buttonWidth - textWidth) / 2;
        int buttonTextY = buttonY + (buttonHeight - 16) / 2;
        tft.setCursor(yesTextX, buttonTextY);
        tft.print(yesText);
        
        // No button (right)
        int noButtonX = dialogX + (dialogWidth / 2) + (buttonSpacing / 2);
        
        updateConfirmNoX = noButtonX;
        updateConfirmNoY = buttonY;
        updateConfirmNoWidth = buttonWidth;
        updateConfirmNoHeight = buttonHeight;
        
        tft.fillRoundRect(noButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_RED);
        tft.drawRoundRect(noButtonX, buttonY, buttonWidth, buttonHeight, 8, TFT_WHITE);
        
        // No button text
        const char* noText = "No";
        textWidth = strlen(noText) * 12;
        int noTextX = noButtonX + (buttonWidth - textWidth) / 2;
        tft.setCursor(noTextX, buttonTextY);
        tft.print(noText);
    }
    
    // Check if display manager is initialized
    bool isInitialized() const {
        return initialized;
    }
    
    // Set flag to update time display
    void setTimeNeedsUpdate(bool needsUpdate) {
        displayNeedsTimeUpdate = needsUpdate;
    }
    
    // Apply current display inversion setting
    void applyDisplayInversion() {
        tft.invertDisplay(isDisplayInverted);
    }
    
    // Get brightness button position
    void getBrightnessButtonArea(int& bX, int& bY, int& bW, int& bH) {
        bX = brightnessButtonX; bY = brightnessButtonY; bW = brightnessButtonWidth; bH = brightnessButtonHeight;
    }
    
    // Get night mode button position
    void getNightModeButtonArea(int& nX, int& nY, int& nW, int& nH) {
        nX = nightModeButtonX; nY = nightModeButtonY; nW = nightModeButtonWidth; nH = nightModeButtonHeight;
    }
    
    // Night mode state management
    bool isNightModeActive() const { return nightModeActive; }
    int getPreviousBrightness() const { return previousBrightness; }
    void setNightMode(bool active, int prevBrightness) {
        nightModeActive = active;
        if (active) {
            previousBrightness = prevBrightness;
        }
    }
    
    // Force redraw settings screen (even if already displaying)
    void forceRedrawSettings() {
        if (settingsPage == 2) {
            drawSettingsPage2();
        } else {
            drawSettingsScreen();
        }
    }
    
    // Get debug button position
    void getDebugButtonArea(int& dX, int& dY, int& dW, int& dH) {
        dX = debugButtonX; dY = debugButtonY; dW = debugButtonWidth; dH = debugButtonHeight;
    }
    
    // Settings page navigation
    int getSettingsPage() { return settingsPage; }
    
    void getSettingsNavDotArea(int& x, int& y, int& size) {
        x = settingsNavDotX; y = settingsNavDotY; size = settingsNavDotSize;
    }
    
    void navigateSettingsPage() {
        if (settingsPage == 1) {
            settingsPage = 2;
            drawSettingsPage2();
        } else {
            settingsPage = 1;
            drawSettingsScreen();
        }
    }
    
    // Show factory reset confirmation dialog
    void showResetConfirmation() {
        displayingConfirmation = true;
        displayNeedsFullUpdate = true;
    }
    
    // WiFi portal flag accessors
    bool shouldLaunchWifiPortal() { return shouldOpenWifiPortal; }
    void clearWifiPortalFlag() { shouldOpenWifiPortal = false; }
    
    // Get brightness screen button areas (minus, plus, save)
    void getBrightnessScreenButtonAreas(int& mX, int& mY, int& mW, int& mH,
                                        int& pX, int& pY, int& pW, int& pH,
                                        int& sX, int& sY, int& sW, int& sH) {
        mX = brightnessMinusX; mY = brightnessMinusY; mW = brightnessMinusWidth; mH = brightnessMinusHeight;
        pX = brightnessPlusX; pY = brightnessPlusY; pW = brightnessPlusWidth; pH = brightnessPlusHeight;
        sX = brightnessSaveX; sY = brightnessSaveY; sW = brightnessSaveWidth; sH = brightnessSaveHeight;
    }
    
    // Set brightness config display state
    void setDisplayingBrightnessConfig(bool displaying) {
        displayingBrightnessConfig = displaying;
        if (displaying) {
            displayNeedsFullUpdate = true;
        }
    }
    
    // Down Detector button area accessor
    void getDownDetectorButtonArea(int& x, int& y, int& w, int& h) {
        x = downDetectorButtonX; y = downDetectorButtonY;
        w = downDetectorButtonWidth; h = downDetectorButtonHeight;
    }
    
    bool isDisplayingDownDetector() const { return displayingDownDetector; }
    
    void setDisplayingDownDetector(bool displaying) {
        displayingDownDetector = displaying;
        if (displaying) displayNeedsFullUpdate = true;
    }
    
    // Draw Down Detector results screen
    // title: screen heading (e.g. "Dexcom Status" or "Libre Status")
    // entries: array of services (label + status string)
    // count: number of entries
    // success: whether the fetch succeeded at all
    void drawDownDetectorScreen(const char* title, const ServiceStatus* entries, int count, bool success) {
        tft.fillScreen(TFT_BLACK);
        
        // Title
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        int tw = strlen(title) * 12;
        tft.setCursor((tft.width() - tw) / 2, 15);
        tft.print(title);
        tft.drawLine(20, 40, tft.width() - 20, 40, TFT_DARKGREY);
        
        if (!success || count <= 0) {
            tft.setTextColor(TFT_RED);
            tft.setCursor(30, 90);
            tft.print("Status check failed");
            tft.setTextSize(1);
            tft.setTextColor(TFT_DARKGREY);
            tft.setCursor(30, 120);
            tft.print("Check WiFi / try again");
        } else {
            // Compute row spacing to fit any number of services between y=55 and y=tft.height()-25
            int firstY = 55;
            int lastY = tft.height() - 30;
            int rowSpacing = (count > 1) ? (lastY - firstY) / (count - 1) : 0;
            // Clamp spacing so rows aren't absurdly far apart with only 2-3 services
            if (rowSpacing > 40) rowSpacing = 40;
            
            for (int i = 0; i < count; i++) {
                int y = firstY + i * rowSpacing;
                const char* label = entries[i].label;
                const char* status = entries[i].status;
                
                tft.setTextSize(2);
                tft.setTextColor(TFT_WHITE);
                tft.setCursor(15, y);
                tft.print(label);
                
                // Determine color/display text for status
                uint16_t color = TFT_WHITE;
                const char* display = status;
                if (strcmp(status, "operational") == 0) {
                    color = TFT_GREEN;
                    display = "OK";
                } else if (strcmp(status, "degraded_performance") == 0) {
                    color = TFT_YELLOW;
                    display = "Degraded";
                } else if (strcmp(status, "partial_outage") == 0) {
                    color = TFT_ORANGE;
                    display = "Partial";
                } else if (strcmp(status, "major_outage") == 0) {
                    color = TFT_RED;
                    display = "OUT";
                } else if (strcmp(status, "under_maintenance") == 0) {
                    color = TFT_CYAN;
                    display = "Maint.";
                } else if (strlen(status) == 0) {
                    color = TFT_DARKGREY;
                    display = "?";
                }
                
                int dw = strlen(display) * 12;
                tft.setTextColor(color);
                tft.setCursor(tft.width() - 15 - dw, y);
                tft.print(display);
            }
        }
        
        // Instruction
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY);
        const char* rt = "Tap anywhere to return";
        int rtw = strlen(rt) * 6;
        tft.setCursor((tft.width() - rtw) / 2, tft.height() - 14);
        tft.print(rt);
    }
    
    void requestBrightnessScreenRefresh() {
        if (displayingBrightnessConfig) {
            displayNeedsFullUpdate = true;
        }
    }
    
    // Check if displaying brightness config
    bool isDisplayingBrightnessConfig() const {
        return displayingBrightnessConfig;
    }
    
    // Draw inline glucose graph with colored dots
    void drawInlineGraph(int centerX, int centerY) {
        if (glucoseHistoryCount == 0) return;
        
        // Graph dimensions - use full display width for better visibility
        int graphWidth = tft.width() - 20;  // Use most of screen width with margins
        int graphHeight = 45;  // Slightly taller for better visibility
        int graphX = 10;  // Start near left edge
        int graphY = centerY - graphHeight / 2;
        
        // Clear the entire graph area first
        tft.fillRect(graphX - 5, graphY - 5, graphWidth + 10, graphHeight + 10, TFT_BLACK);
        
        // Determine how many readings to show (max 24 for ~2 hours)
        int readingsToShow = min(glucoseHistoryCount, 24);
        if (readingsToShow < 2) return; // Need at least 2 points
        
        // Find min/max glucose values for scaling
        int minGlucose = (int)glucoseHistory[0].value;
        int maxGlucose = (int)glucoseHistory[0].value;
        for (int i = 1; i < readingsToShow; i++) {
            int currentGlucose = (int)glucoseHistory[i].value;
            if (currentGlucose < minGlucose) minGlucose = currentGlucose;
            if (currentGlucose > maxGlucose) maxGlucose = currentGlucose;
        }
        
        // Make graph MORE exaggerated to show dramatic changes like 60-point drops
        int range = maxGlucose - minGlucose;
        
        // Reduce padding even further to make graph more sensitive to changes
        int padding = max(5, range / 30); // Even less padding: 3.3% of range or minimum 5 mg/dL
        
        // Apply minimal padding to preserve glucose variations
        minGlucose -= padding;
        maxGlucose += padding;
        range = maxGlucose - minGlucose;
        
        // Use even smaller minimum range to show MORE variation and detail
        if (range < 25) { // Reduced from 40 to 25 mg/dL for maximum sensitivity
            int center = (minGlucose + maxGlucose) / 2;
            minGlucose = center - 12;
            maxGlucose = center + 13; // 25 mg/dL total range
            range = 25;
        }
        
        // Prevent negative glucose values in display
        if (minGlucose < 0) {
            maxGlucose += (0 - minGlucose);
            minGlucose = 0;
        }
        
        // Draw dots for each glucose reading
        // CRITICAL FIX: Reverse order so oldest is on LEFT, newest is on RIGHT (like Dexcom app)
        for (int i = 0; i < readingsToShow; i++) {
            // Use reverse index: oldest readings (higher index) go to the left
            int historyIndex = readingsToShow - 1 - i;
            int glucose = (int)glucoseHistory[historyIndex].value;
            
            // Calculate position - i=0 is leftmost (oldest), i=readingsToShow-1 is rightmost (newest)
            int dotX = graphX + (i * graphWidth) / (readingsToShow - 1);
            int dotY = graphY + graphHeight - ((glucose - minGlucose) * graphHeight) / range;
            
            // Ensure dot stays within bounds
            dotX = constrain(dotX, graphX, graphX + graphWidth);
            dotY = constrain(dotY, graphY, graphY + graphHeight);
            
            // Dynamic dot coloring - each dot gets colored based on its individual glucose value
            // EXACT SAME LOGIC AS MAIN GLUCOSE DISPLAY
            uint16_t dotColor;
            
            // Determine color based on glucose thresholds (identical to main display logic)
            if (glucose < 40) {
                dotColor = COLOR_BG_LOW;    // Always RED for very low (< 40 mg/dL)
            } else if (glucose >= 401) {
                dotColor = COLOR_BG_HIGH;   // Always YELLOW for very high (>= 401 mg/dL)  
            } else if (glucose < settings.lowThreshold) {
                dotColor = COLOR_BG_LOW;    // Below lowThreshold = RED (low)
            } else if (glucose > settings.highThreshold) {
                dotColor = COLOR_BG_HIGH;   // Above highThreshold = YELLOW (high)
            } else {
                dotColor = COLOR_BG_NORMAL; // Between thresholds = GREEN (good)
            }
            
            // Debug output for color mismatch troubleshooting
            if (i == readingsToShow - 1) { // Only debug the newest (rightmost) dot
                Serial.print("[DEBUG] Newest dot: glucose="); Serial.print(glucose);
                Serial.print(", lowThresh="); Serial.print(settings.lowThreshold);
                Serial.print(", highThresh="); Serial.print(settings.highThreshold);
                Serial.print(", color=");
                if (dotColor == COLOR_BG_LOW) Serial.println("RED");
                else if (dotColor == COLOR_BG_NORMAL) Serial.println("GREEN");
                else if (dotColor == COLOR_BG_HIGH) Serial.println("YELLOW");
                else Serial.println("UNKNOWN");
            }
            
            // Draw smaller dot for more detail - reduced from 4 to 3 pixel radius
            tft.fillCircle(dotX, dotY, 3, dotColor);
        }
        
        // Label the time span the graph covers, so the two views (short vs long) are
        // distinguishable at a glance. Prefer the readings' real timestamps (newest is index 0,
        // oldest shown is index readingsToShow-1) since the actual per-dot spacing differs by
        // provider (e.g. Dexcom is ~5 min native). Fall back to the requested interval if
        // timestamps are unavailable.
        int spanMin;
        unsigned long newestTs = glucoseHistory[0].timestamp;
        unsigned long oldestTs = glucoseHistory[readingsToShow - 1].timestamp;
        if (newestTs > oldestTs && oldestTs > 0) {
            spanMin = (int)((newestTs - oldestTs) / 60UL);
        } else {
            spanMin = (readingsToShow - 1) * inlineGraphIntervalMin;
        }
        char spanLabel[10];
        if (spanMin >= 90) {
            snprintf(spanLabel, sizeof(spanLabel), "~%dh", (spanMin + 30) / 60);
        } else {
            snprintf(spanLabel, sizeof(spanLabel), "~%dm", spanMin);
        }
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawString(spanLabel, graphX, graphY - 4);
    }
    
    // Inline graph management methods
    void toggleInlineGraph(bool show) {
        if (displayingInlineGraph != show) {
            displayingInlineGraph = show;
            displayNeedsFullUpdate = true;
        }
    }
    
    // Cycle the inline graph: off -> dense (1 min/dot, ~recent) -> wide (15 min/dot, ~6h) -> off.
    // Returns true if the graph is now visible, so the caller can (re)fetch history at the new
    // interval. Always flags a full redraw so the new dots/label paint immediately.
    bool cycleInlineGraph() {
        if (!displayingInlineGraph) {
            displayingInlineGraph = true;
            inlineGraphIntervalMin = 1;
        } else if (inlineGraphIntervalMin == 1) {
            inlineGraphIntervalMin = 15;
        } else {
            displayingInlineGraph = false;
            inlineGraphIntervalMin = 1;
        }
        displayNeedsFullUpdate = true;
        return displayingInlineGraph;
    }
    
    bool isDisplayingInlineGraph() const {
        return displayingInlineGraph;
    }
    
    int getInlineGraphIntervalMin() const {
        return inlineGraphIntervalMin;
    }
};

// Global display manager
DisplayManager displayManager;

// Diagnostic bridge for DexcomClient (avoids header dependency on DisplayManager)
void setDiagnosticStatePublicInt(int state) {
    switch (state) {
        case 0: // OK
            displayManager.clearDiagnosticState();
            break;
        case 1: // WIFI_DISCONNECTED
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_WIFI_DISCONNECTED_PUBLIC);
            break;
        case 2: // MISSING_CREDENTIALS
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_MISSING_CREDENTIALS_PUBLIC);
            break;
        case 3: // AUTH_FAILED
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_AUTH_FAILED_PUBLIC);
            break;
        case 4: // LOGIN_FAILED
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_LOGIN_FAILED_PUBLIC);
            break;
        case 5: // SESSION_EXPIRED
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_SESSION_EXPIRED_PUBLIC);
            break;
        case 6: // SERVER_ERROR
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_SERVER_ERROR_PUBLIC);
            break;
        case 7: // CONNECTION_ERROR
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_CONNECTION_ERROR_PUBLIC);
            break;
        case 8: // DATA_STALE
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_DATA_STALE_PUBLIC);
            break;
        default:
            // Unknown code, treat as generic error
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_SERVER_ERROR_PUBLIC);
            break;
    }
}

// Touch handler class
class TouchHandler {
private:
    bool touchEnabled = false;
    unsigned long lastTouchTime = 0;
    bool waitForRelease = false; // Block touches until finger is lifted after screen transition
    
public:
    bool init() {
        touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
        if (touchscreen.begin(touchscreenSPI)) {
            Serial.println("Touch screen initialized");
            touchEnabled = true;
            touchscreen.setRotation(1); // Match TFT rotation
            return true;
        } else {
            Serial.println("Touch screen initialization failed");
            return false;
        }
    }
    
    // DEDICATED BRIGHTNESS SCREEN TOUCH HANDLER - NO SETTINGS LOGIC
    void handleBrightnessScreenTouch(int x, int y) {
        // Use actual button coordinates from DisplayManager (set during drawBrightnessConfigScreen)
        int brightnessMinusX, brightnessMinusY, brightnessMinusWidth, brightnessMinusHeight;
        int brightnessPlusX, brightnessPlusY, brightnessPlusWidth, brightnessPlusHeight;
        int brightnessSaveX, brightnessSaveY, brightnessSaveWidth, brightnessSaveHeight;
        
        displayManager.getBrightnessScreenButtonAreas(
            brightnessMinusX, brightnessMinusY, brightnessMinusWidth, brightnessMinusHeight,
            brightnessPlusX, brightnessPlusY, brightnessPlusWidth, brightnessPlusHeight,
            brightnessSaveX, brightnessSaveY, brightnessSaveWidth, brightnessSaveHeight
        );
        
        // Check if minus button was pressed
        if (x >= brightnessMinusX && x <= (brightnessMinusX + brightnessMinusWidth) && 
            y >= brightnessMinusY && y <= (brightnessMinusY + brightnessMinusHeight)) {
            
            Serial.println("MINUS BUTTON HIT!");
            
            // Decrease brightness by 10% using clean percentage steps
            int currentPercent = (settings.brightness * 100 + 127) / 255;
            currentPercent = ((currentPercent + 5) / 10) * 10;
            
            int newPercent = currentPercent - 10;
            if (newPercent < 10) newPercent = 10;
            
            int newBrightness = (newPercent * 255 + 50) / 100;
            settings.brightness = newBrightness;
            
            setBrightness(settings.brightness);
            
            // Force display refresh to show new percentage
            Serial.print("Brightness decreased to: ");
            Serial.print(newPercent);
            Serial.println("%");
            
            displayManager.requestBrightnessScreenRefresh();
            return;
        }
        
        // Check if plus button was pressed
        if (x >= brightnessPlusX && x <= (brightnessPlusX + brightnessPlusWidth) && 
            y >= brightnessPlusY && y <= (brightnessPlusY + brightnessPlusHeight)) {
            
            Serial.println("PLUS BUTTON HIT!");
            
            // Increase brightness by 10% using clean percentage steps
            int currentPercent = (settings.brightness * 100 + 127) / 255;
            currentPercent = ((currentPercent + 5) / 10) * 10;
            
            int newPercent = currentPercent + 10;
            if (newPercent > 100) newPercent = 100;
             
            int newBrightness = (newPercent * 255 + 50) / 100;
            settings.brightness = newBrightness;
            
            setBrightness(settings.brightness);
            
            // Force display refresh to show new percentage
            Serial.print("Brightness increased to: ");
            Serial.print(newPercent);
            Serial.println("%");
            
            displayManager.requestBrightnessScreenRefresh();
            return;
        }
        
        // Check if save button was pressed
        if (x >= brightnessSaveX && x <= (brightnessSaveX + brightnessSaveWidth) && 
            y >= brightnessSaveY && y <= (brightnessSaveY + brightnessSaveHeight)) {
            
            Serial.println("SAVE BUTTON HIT!");
            
            Serial.println("Save button pressed - saving settings and exiting");
            settings.save();
            displayManager.setDisplayingBrightnessConfig(false);
            displayManager.toggleSettings(true);
            waitForRelease = true; // Block touches until finger is lifted
            return;
        }
        
        // If touch outside all buttons, KEEP brightness screen open and consume the touch
        // This prevents touches from exiting and then triggering underlying settings buttons.
        Serial.println("Touch outside brightness controls - ignored");
        return;
    }
    
    // IMPROVED TOUCH HANDLING
    bool handleTouch(unsigned long currentTime) {
        if (!touchEnabled) return false;
        
        // If waiting for finger release, check if touch is no longer active
        if (waitForRelease) {
            if (!touchscreen.tirqTouched() || !touchscreen.touched()) {
                waitForRelease = false;
                lastTouchTime = millis(); // Start debounce from release point
            }
            return false; // Block all touches until released
        }
        
        // Check if touch is detected and debounce it
        // Note: lastTouchTime may be set to a future value (millis() + N) for extra cooldown
        // so we must check that currentTime is actually past lastTouchTime before subtracting
        if (touchscreen.tirqTouched() && touchscreen.touched() && 
            currentTime > lastTouchTime && (currentTime - lastTouchTime) > TOUCH_DEBOUNCE) {
            
            TS_Point p = touchscreen.getPoint();
            
            // Convert touch coordinates to screen coordinates
            // IMPORTANT: These mapping values may need adjustment for your specific hardware
            int touchX = map(p.x, 200, 3700, 0, tft.width());
            int touchY = map(p.y, 240, 3800, 0, tft.height());
            
            // Print touch coordinates for debugging
            if (DEBUG_TOUCH) {
                Serial.print("Touch detected at X=");
                Serial.print(touchX);
                Serial.print(", Y=");
                Serial.print(touchY);
                Serial.print(", Pressure=");
                Serial.println(p.z);
            }
            
            // PRIORITY 1: Check if we're on brightness screen first - handle brightness touches immediately
            if (displayManager.isDisplayingBrightnessConfig()) {
                // Handle ONLY brightness screen touches - bypass all settings logic completely
                Serial.printf("TOUCH: brightness screen active, handling at (%d,%d)\n", touchX, touchY);
                handleBrightnessScreenTouch(touchX, touchY);
                // Use millis() (not cached currentTime) because save+redraw can take 200-400ms
                // This prevents the still-held finger from registering as a new touch on the next screen
                lastTouchTime = millis() + 300;
                return false;  // Block all other touch processing when on brightness screen
            }
            
            // PRIORITY 1b: Down Detector screen - any tap returns to settings page 2
            if (displayManager.isDisplayingDownDetector()) {
                Serial.println("TOUCH: down detector active - returning to settings");
                displayManager.setDisplayingDownDetector(false);
                displayManager.toggleSettings(true);
                // Force back to page 2
                if (displayManager.getSettingsPage() != 2) {
                    displayManager.navigateSettingsPage();
                }
                lastTouchTime = millis() + 300;
                return false;
            }

            // PRIORITY 2: If an update confirmation dialog is visible, handle ONLY it
            if (displayManager.isDisplayingUpdateConfirmation()) {
                int yesX, yesY, yesW, yesH, noX, noY, noW, noH;
                displayManager.getUpdateConfirmationButtonAreas(yesX, yesY, yesW, yesH, noX, noY, noW, noH);

                // Yes - check for update (don't download yet)
                if (touchX >= yesX && touchX <= (yesX + yesW) && touchY >= yesY && touchY <= (yesY + yesH)) {
                    if (DEBUG_TOUCH) Serial.println("Update confirmation Yes (modal) tapped - checking for update");
                    displayManager.setDisplayingUpdateConfirmation(false);
                    
                    // Show "Checking..." message
                    tft.fillScreen(TFT_BLACK);
                    tft.setTextSize(2);
                    tft.setTextColor(TFT_WHITE);
                    tft.setCursor(60, 100);
                    tft.println("Checking...");
                    
                    // Check if an update is available
                    char remoteVersion[16];
                    int updateResult = displayManager.checkForUpdate(remoteVersion, sizeof(remoteVersion));
                    
                    if (updateResult == 1) {
                        // Update available - show download confirmation dialog
                        displayManager.setDisplayingDownloadConfirmation(true);
                        displayManager.drawDownloadConfirmation();
                    } else if (updateResult == 0) {
                        // No update available
                        displayManager.showUpdateCheckResult(updateResult, remoteVersion);
                        delay(3000);
                        displayManager.toggleSettings(true);
                    } else {
                        // Error checking for updates
                        displayManager.showUpdateCheckResult(updateResult, remoteVersion);
                        delay(3000);
                        displayManager.toggleSettings(true);
                    }
                    lastTouchTime = currentTime;
                    return false; // Consume touch exclusively
                }

                // No
                if (touchX >= noX && touchX <= (noX + noW) && touchY >= noY && touchY <= (noY + noH)) {
                    if (DEBUG_TOUCH) Serial.println("Update confirmation No (modal) tapped");
                    // Hides the dialog and internally flags a full redraw
                    displayManager.setDisplayingUpdateConfirmation(false);
                    lastTouchTime = currentTime;
                    return false; // Consume touch exclusively
                }

                // Tapped outside dialog: ignore and do not let it pass through
                lastTouchTime = currentTime;
                return false;
            }

            // PRIORITY 2.5: If download confirmation dialog is visible, handle ONLY it
            if (displayManager.isDisplayingDownloadConfirmation()) {
                int yesX, yesY, yesW, yesH, noX, noY, noW, noH;
                displayManager.getDownloadConfirmationButtonAreas(yesX, yesY, yesW, yesH, noX, noY, noW, noH);

                // Update button - proceed with download
                if (touchX >= yesX && touchX <= (yesX + yesW) && touchY >= yesY && touchY <= (yesY + yesH)) {
                    if (DEBUG_TOUCH) Serial.println("Download confirmation Update tapped - starting download");
                    displayManager.setDisplayingDownloadConfirmation(false);
                    
                    if (displayManager.performOtaUpdate()) {
                        // Device will restart on success
                    } else {
                        tft.fillScreen(TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_RED);
                        tft.setCursor(20, 60);
                        tft.println("Update failed!");
                        tft.setCursor(20, 90);
                        tft.println("Tap to return");
                        delay(3000);
                        displayManager.toggleSettings(true);
                    }
                    lastTouchTime = currentTime;
                    return false;
                }

                // Cancel button - go back to settings
                if (touchX >= noX && touchX <= (noX + noW) && touchY >= noY && touchY <= (noY + noH)) {
                    if (DEBUG_TOUCH) Serial.println("Download confirmation Cancel tapped");
                    displayManager.setDisplayingDownloadConfirmation(false);
                    displayManager.toggleSettings(true);
                    lastTouchTime = currentTime;
                    return false;
                }

                // Tapped outside dialog: ignore
                lastTouchTime = currentTime;
                return false;
            }

            // PRIORITY 3: If a reset confirmation dialog is visible, handle ONLY it
            if (displayManager.isDisplayingConfirmation()) {
                bool shouldRestart = displayManager.handleResetCredentialsTouch(touchX, touchY);
                lastTouchTime = currentTime;
                return shouldRestart; // Consume; may request restart
            }

            // Get touch areas
            int gX, gY, gW, gH, aX, aY, aW, aH, tX, tY, tW, tH, sX, sY, sW, sH;
            displayManager.getTouchAreas(gX, gY, gW, gH, aX, aY, aW, aH, 
                                      tX, tY, tW, tH, sX, sY, sW, sH);
            
            // Check for reset credentials actions (only if NOT on brightness screen and NOT while any confirmation is active)
            if (!displayManager.isDisplayingBrightnessConfig() && 
                !displayManager.isDisplayingUpdateConfirmation() && 
                !displayManager.isDisplayingDownloadConfirmation() &&
                displayManager.handleResetCredentialsTouch(touchX, touchY)) {
                return true;  // Return true to indicate we should restart
            }
            
            // 1. First check if we're in settings screen (but NOT brightness screen)
            if (displayManager.isDisplayingSettings() && !displayManager.isDisplayingBrightnessConfig()) {
                // Page 1-only buttons: Update, Brightness, Night Mode
                if (displayManager.getSettingsPage() == 1) {
                // Check if update button was tapped
                int updateX, updateY, updateW, updateH;
                displayManager.getUpdateButtonArea(updateX, updateY, updateW, updateH);
                
                if (touchX >= updateX && touchX <= (updateX + updateW) && touchY >= updateY && touchY <= (updateY + updateH)) {
                    if (DEBUG_TOUCH) Serial.println("Update button tapped");
                    // Show update confirmation dialog
                    displayManager.setDisplayingUpdateConfirmation(true);
                    lastTouchTime = currentTime;
                    return false;
                }
                
                // Check if brightness button was tapped
                int brightX, brightY, brightW, brightH;
                displayManager.getBrightnessButtonArea(brightX, brightY, brightW, brightH);
                
                if (touchX >= brightX && touchX <= (brightX + brightW) && touchY >= brightY && touchY <= (brightY + brightH)) {
                    if (DEBUG_TOUCH) Serial.println("Brightness button tapped");
                    // Show brightness adjustment screen
                    displayManager.setDisplayingBrightnessConfig(true);
                    // Extra cooldown to prevent held finger from hitting Save on the new screen
                    lastTouchTime = millis() + 300;
                    return false;
                }
                
                // Check if night mode button was tapped
                int nightX, nightY, nightW, nightH;
                displayManager.getNightModeButtonArea(nightX, nightY, nightW, nightH);
                
                if (touchX >= nightX && touchX <= (nightX + nightW) && touchY >= nightY && touchY <= (nightY + nightH)) {
                    if (DEBUG_TOUCH) Serial.println("Night Mode button tapped");
                    
                    if (!displayManager.isNightModeActive()) {
                        // Activate night mode - save current brightness and set to 1%
                        displayManager.setNightMode(true, settings.brightness);
                        int nightBrightness = (1 * 255 + 50) / 100;  // 1% brightness
                        settings.brightness = nightBrightness;
                        setBrightness(settings.brightness);
                        settings.save();
                        Serial.println("Night Mode activated - brightness set to 1%");
                    } else {
                        // Deactivate night mode - restore previous brightness
                        int prevBrightness = displayManager.getPreviousBrightness();
                        displayManager.setNightMode(false, 0);
                        settings.brightness = prevBrightness;
                        setBrightness(settings.brightness);
                        settings.save();
                        Serial.printf("Night Mode deactivated - brightness restored to %d%%\n", (prevBrightness * 100) / 255);
                    }
                    
                    // Redraw settings to update button color
                    displayManager.forceRedrawSettings();
                    lastTouchTime = currentTime;
                    return false;
                }
                } // end page 1 buttons
                
                // Page 2-only buttons
                if (displayManager.getSettingsPage() == 2) {
                    int ddX, ddY, ddW, ddH;
                    displayManager.getDownDetectorButtonArea(ddX, ddY, ddW, ddH);
                    if (touchX >= ddX && touchX <= (ddX + ddW) && touchY >= ddY && touchY <= (ddY + ddH)) {
                        if (DEBUG_TOUCH) Serial.println("Down Detector button tapped");
                        
                        // Show "Checking..." message
                        tft.fillScreen(TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_WHITE);
                        const char* msg = "Checking status...";
                        int mw = strlen(msg) * 12;
                        tft.setCursor((tft.width() - mw) / 2, 100);
                        tft.print(msg);
                        
                        // Pick provider based on which credentials are configured
                        bool libreUser = isLibreUser();
                        const char* title = libreUser ? "Libre Status" : "Dexcom Status";
                        
                        // Fetch status - up to 4 services (Libre case)
                        ServiceStatus entries[4];
                        int count = 0;
                        bool ok = libreUser
                            ? fetchLibreStatus(entries, 4, count)
                            : fetchDexcomStatus(entries, 4, count);
                        
                        // Enter down-detector display state and render results
                        displayManager.toggleSettings(false); // we will draw a custom screen
                        displayManager.setDisplayingDownDetector(true);
                        displayManager.drawDownDetectorScreen(title, entries, count, ok);
                        // Skip the main-loop redraw cycle for this custom screen
                        displayManager.clearFullUpdate();
                        
                        lastTouchTime = millis() + 300;
                        return false;
                    }
                }
                
                // Check if navigation dot was tapped (top-right corner) - both pages
                int navDotX, navDotY, navDotSize;
                displayManager.getSettingsNavDotArea(navDotX, navDotY, navDotSize);
                
                if (touchX >= (navDotX - navDotSize) && touchX <= (navDotX + navDotSize) &&
                    touchY >= (navDotY - navDotSize) && touchY <= (navDotY + navDotSize)) {
                    if (DEBUG_TOUCH) Serial.println("Settings nav dot tapped");
                    displayManager.navigateSettingsPage();
                    lastTouchTime = millis() + 300;
                    return false;
                }
                
                // Check if debug/reset button was tapped (position depends on page)
                int dbgX, dbgY, dbgW, dbgH;
                displayManager.getDebugButtonArea(dbgX, dbgY, dbgW, dbgH);
                
                if (touchX >= dbgX && touchX <= (dbgX + dbgW) && touchY >= dbgY && touchY <= (dbgY + dbgH)) {
                    if (displayManager.getSettingsPage() == 1) {
                        // Page 1: This is the Reset (factory reset) button
                        Serial.printf("TOUCH: Reset button HIT at (%d,%d) - btn area: x=%d y=%d w=%d h=%d, brightness=%d settings=%d\n",
                                      touchX, touchY, dbgX, dbgY, dbgW, dbgH,
                                      displayManager.isDisplayingBrightnessConfig(), displayManager.isDisplayingSettings());
                        displayManager.showResetConfirmation();
                        lastTouchTime = currentTime;
                        return false;
                    } else {
                        // Page 2: This is the Debug button
                        if (DEBUG_TOUCH) Serial.println("Debug button tapped");
                        
                        // Show sending status on screen
                        tft.fillScreen(TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setCursor(40, 60);
                        tft.println("Sending debug log...");
                        tft.setTextSize(1);
                        tft.setTextColor(TFT_CYAN, TFT_BLACK);
                        tft.setCursor(40, 100);
                        tft.printf("Buffer: %d bytes", debugLogCount);
                        
                        bool success = sendDebugLog();
                        
                        tft.fillScreen(TFT_BLACK);
                        tft.setTextSize(2);
                        if (success) {
                            tft.setTextColor(TFT_GREEN, TFT_BLACK);
                            tft.setCursor(60, 80);
                            tft.println("Log sent!");
                        } else {
                            tft.setTextColor(TFT_RED, TFT_BLACK);
                            tft.setCursor(50, 80);
                            tft.println("Send failed!");
                        }
                        delay(1500);
                        displayManager.toggleSettings(false); // Return to glucose screen
                        lastTouchTime = currentTime;
                        return false;
                    }
                }
                
                // Update confirmation modal is handled earlier (PRIORITY 2) to avoid
                // processing it within the same touch that opened it. No handling here.
                
                // Check if any settings button was tapped (prevent closing settings)
                int resetX, resetY, resetW, resetH;
                displayManager.getResetButtonArea(resetX, resetY, resetW, resetH);
                int invX, invY, invW, invH;
                displayManager.getInvertButtonArea(invX, invY, invW, invH);
                int rotX, rotY, rotW, rotH;
                displayManager.getRotateButtonArea(rotX, rotY, rotW, rotH);
                
                bool tappedSettingsButton = false;
                
                if (displayManager.getSettingsPage() == 1) {
                    // Page 1: WiFi, Night Mode, Invert, Rotate, Reset, Update, Brightness buttons
                    tappedSettingsButton = (touchX >= resetX && touchX <= (resetX + resetW) && touchY >= resetY && touchY <= (resetY + resetH)) ||
                                            (touchX >= invX && touchX <= (invX + invW) && touchY >= invY && touchY <= (invY + invH)) ||
                                            (touchX >= rotX && touchX <= (rotX + rotW) && touchY >= rotY && touchY <= (rotY + rotH));
                } else {
                    // Page 2: Debug button
                    tappedSettingsButton = (touchX >= dbgX && touchX <= (dbgX + dbgW) && touchY >= dbgY && touchY <= (dbgY + dbgH));
                }
                
                if (tappedSettingsButton) {
                    // These are handled above - don't close settings
                    lastTouchTime = currentTime;
                    return false;
                }
                
                // Check if settings button was tapped - toggle settings off
                if (touchX >= sX && touchX <= (sX + sW) && touchY >= sY && touchY <= (sY + sH)) {
                    if (DEBUG_TOUCH) Serial.println("Settings button tapped while in settings");
                    displayManager.toggleSettings(false);
                } else if (!displayManager.isDisplayingConfirmation() && !displayManager.isDisplayingUpdateConfirmation() && !displayManager.isDisplayingDownloadConfirmation()) {
                    // Any other tap while in settings returns to main screen (as long as confirmation dialog isn't showing)
                    if (DEBUG_TOUCH) Serial.println("Screen tapped while in settings - returning to main");
                    displayManager.toggleSettings(false);
                }
                lastTouchTime = currentTime;
                return false;
            }
            
            // 3. Next, check if we're in keyboard screen - need to handle keyboard input or exit
            // No keyboard handling for now
            
            // 4. We're in main screen - check if settings button was tapped
            if (touchX >= sX && touchX <= (sX + sW) && touchY >= sY && touchY <= (sY + sH)) {
                if (DEBUG_TOUCH) Serial.println("Settings button tapped");
                displayManager.toggleSettings(true);
                lastTouchTime = currentTime;
                return false;
            }
            
            // 5. Check if glucose display area was tapped
            if (touchX >= gX && touchX <= (gX + gW) && touchY >= gY && touchY <= (gY + gH)) {
                if (DEBUG_TOUCH) Serial.println("Glucose area tapped - toggling units");
                displayManager.toggleDisplayMode();
                lastTouchTime = currentTime;
                return false;
            }
            
            // 6. Check if arrow/trend area was tapped
            if (touchX >= aX && touchX <= (aX + aW) && touchY >= aY && touchY <= (aY + aH)) {
                if (DEBUG_TOUCH) Serial.println("Arrow area tapped - cycling inline graph mode");
                bool nowShowing = displayManager.cycleInlineGraph();
                
                // The graph mode changed, so we must (re)sample the history at the new interval.
                // Bypass the normal cooldown here: this is a deliberate user tap and the two views
                // are sampled differently from the (same) underlying data.
                if (nowShowing) {
                    int interval = displayManager.getInlineGraphIntervalMin();
                    int count = 0;
                    bool gotHistory = isLibreUser()
                        ? libreClient.fetchGlucoseHistory(glucoseHistory, count, MAX_GLUCOSE_HISTORY, interval)
                        : dexcomClient.fetchGlucoseHistory(glucoseHistory, count, interval);
                    if (gotHistory) {
                        glucoseHistoryCount = count;
                        g_lastHistoryFetchTime = currentTime;
                    }
                }
                
                lastTouchTime = currentTime;
                return false;
            }
            
            // 8. Title area tap no longer does anything
            
            // 9. Check if time area at bottom of screen was tapped
            int timeXArea, timeYArea, timeWArea, timeHArea;
            displayManager.getTimeTouchArea(timeXArea, timeYArea, timeWArea, timeHArea);
            if (touchX >= timeXArea && touchX <= (timeXArea + timeWArea) &&
                touchY >= timeYArea && touchY <= (timeYArea + timeHArea)) {
                if (DEBUG_TOUCH) Serial.println("Time area tapped - toggling time/date display");
                displayManager.toggleTimeDisplay();
                lastTouchTime = currentTime;
                return false;
            }
            
            // Wait a bit to debounce
            delay(50);
        }
        
        return false;
    }
    
    bool isTouchEnabled() const {
        return touchEnabled;
    }
};

// Global touch handler
TouchHandler touchHandler;

// WiFi manager helper class
class WifiHelper {
public:
    static void configModeCallback(WiFiManager *myWiFiManager) {
        Serial.println("Entered config mode");
        displayManager.showConfigMode();
    }

    bool connect() {
        // Try ALL stored WiFi networks first. WiFiManager.autoConnect() (below) only remembers the
        // single last-connected AP in the ESP32's own NVS, so after moving between networks (e.g.
        // home vs a vacation hotel) it would fail and drop into the setup portal even when another
        // known network is in range. wifiMulti was already populated from settings in setup(), so
        // run() scans and joins the strongest available known network. Skipped on a settings reset.
        if (!shouldResetSettings && settings.wifiNetworkCount > 0) {
            Serial.printf("Trying %d stored WiFi network(s) before setup portal...\n",
                          settings.wifiNetworkCount);
            WiFi.mode(WIFI_STA);
            if (wifiMulti.run(20000) == WL_CONNECTED) {
                Serial.printf("Connected to stored network: %s\n", WiFi.SSID().c_str());
                // Keep this network in the stored list (dedups / refreshes its password).
                String connectedSSID = WiFi.SSID();
                String connectedPSK = WiFi.psk();
                if (connectedSSID.length() > 0) {
                    settings.addWifiNetwork(connectedSSID.c_str(), connectedPSK.c_str());
                }
                tft.fillScreen(TFT_BLACK);
                tft.setTextSize(2);
                tft.setCursor(20, 60);
                tft.println("WiFi Connected!");
                tft.setCursor(20, 90);
                tft.println("IP: " + WiFi.localIP().toString());
                delay(1500);
                return true;
            }
            Serial.println("No stored network reachable - falling back to setup portal");
        }
        
        WiFiManager wifiManager;
        
        // Set callback for config mode
        wifiManager.setAPCallback(configModeCallback);
        
        // Optional device name shown at the top of the main screen (useful when one person
        // owns multiple SugrBee devices for different family members). Leave blank to hide.
        WiFiManagerParameter custom_html_devicename_label("<p style='padding-top: 5px; padding-bottom: 0px;'><b>Device Name (optional)</b><br><span style='font-size:0.8em;color:#888;'>Shown at top of screen - leave blank to hide</span></p>");
        WiFiManagerParameter custom_device_name("deviceName", "Device Name", settings.deviceName, 23);
        wifiManager.addParameter(&custom_html_devicename_label);
        wifiManager.addParameter(&custom_device_name);
        
        // Bolded section header so Dexcom Credentials is visually separated from Device Name above
        WiFiManagerParameter custom_html_dexcom_label("<p style='padding-top: 10px; padding-bottom: 0px;'><b>Dexcom Credentials</b></p>");
        wifiManager.addParameter(&custom_html_dexcom_label);
        
        // Add custom parameters for Dexcom credentials
        WiFiManagerParameter custom_dexcom_username("dexcomUser", "Dexcom Username", settings.dexcomUsername, 50);
        WiFiManagerParameter custom_dexcom_password("dexcomPass", "Dexcom Password", settings.dexcomPassword, 50);
        
        // Server region dropdown
        char serverRegionStr[2];
        sprintf(serverRegionStr, "%d", settings.serverRegion);
        
        // Custom HTML dropdown for server selection. WiFiManagerParameter has no native <select>
        // support, so this renders as raw, untracked HTML - it is NOT read back on save. Its id is
        // 'serverRegionSelect' (deliberately NOT 'serverRegion') so it can't collide with the
        // tracked hidden field below. The inline onchange handler copies the chosen value into that
        // hidden field, which IS what actually gets saved.
        char serverSelectHtml[480];
        snprintf(serverSelectHtml, sizeof(serverSelectHtml),
            "<br><label>Dexcom Server Region</label>"
            "<select id='serverRegionSelect' onchange=\"document.getElementById('serverRegion').value=this.value;\" style='width:100%%;padding:5px;'>"
            "<option value='0'%s>US (United States)</option>"
            "<option value='1'%s>EU / Outside US</option>"
            "<option value='2'%s>Japan</option>"
            "</select>",
            settings.serverRegion == 0 ? " selected" : "",
            settings.serverRegion == 1 ? " selected" : "",
            settings.serverRegion == 2 ? " selected" : "");
        
        WiFiManagerParameter custom_server_select(serverSelectHtml);
        // Hidden input that actually gets read back on save (see addParameter below). Previously
        // this was constructed but never registered with wifiManager.addParameter(), so WiFiManager
        // never updated it from the submitted form - its value stayed frozen at whatever
        // settings.serverRegion was BEFORE the portal opened, no matter what the user picked above.
        // That made the dropdown silently cosmetic: selecting "EU" never actually changed the saved
        // region or which Dexcom server was used.
        WiFiManagerParameter custom_server_region("serverRegion", "", serverRegionStr, 2, "type='hidden'");
        
        wifiManager.addParameter(&custom_dexcom_username);
        wifiManager.addParameter(&custom_dexcom_password);
        wifiManager.addParameter(&custom_server_select);
        wifiManager.addParameter(&custom_server_region);
        
        // --- Add LibreView Parameters --- 
        // Bolded section header + BETA note as sub-line
        WiFiManagerParameter custom_html_libre_label("<p style='padding-top: 10px; padding-bottom: 0px;'><b>LibreView Credentials</b><br><span style='font-size:0.8em;color:#888;'><i>LibreView integration is BETA</i></span></p>");
        wifiManager.addParameter(&custom_html_libre_label);
        
        // LibreView Email Input
        WiFiManagerParameter custom_libreview_email("libreEmail", "LibreView Email", settings.libreViewEmail, 50);
        wifiManager.addParameter(&custom_libreview_email);
        
        // LibreView Password Input
        WiFiManagerParameter custom_libreview_password("librePass", "LibreView Password", settings.libreViewPassword, 50, "type='password'"); // Mask password input
        wifiManager.addParameter(&custom_libreview_password);
        
        // LibreView Server Region - auto-detected on first login
        // Show current region if set, or "Auto-detect" if not
        char libreRegionInfoHtml[256];
        if (strlen(settings.libreViewRegion) > 0) {
            snprintf(libreRegionInfoHtml, sizeof(libreRegionInfoHtml),
                "<br><label><b>LibreView Region</b></label>"
                "<p style='font-size:0.9em;color:#4CAF50;'>Auto-detected: <b>%s</b> (will re-detect on login)</p>",
                settings.libreViewRegion);
        } else {
            snprintf(libreRegionInfoHtml, sizeof(libreRegionInfoHtml),
                "<br><label><b>LibreView Region</b></label>"
                "<p style='font-size:0.9em;color:#888;'>Will auto-detect on first login</p>");
        }
        WiFiManagerParameter custom_libre_region_info(libreRegionInfoHtml);
        wifiManager.addParameter(&custom_libre_region_info);
        // --- End LibreView Parameters ---
        
        // --- Add Glucose Threshold Parameters ---
        // Glucose Thresholds Label
        WiFiManagerParameter custom_html_thresholds_label("<p style='padding-top: 10px; padding-bottom: 0px;'><b>Glucose Color Thresholds</b></p>");
        wifiManager.addParameter(&custom_html_thresholds_label);
        
        // Information about fixed extreme values
        WiFiManagerParameter custom_html_threshold_info("<p style='font-size: 0.8em; color: #888;'>LOW=RED, GOOD=GREEN, HIGH=YELLOW These colors will be reflected on your glucose values and graph</p>");
        wifiManager.addParameter(&custom_html_threshold_info);
        
        // Convert thresholds to strings for the input fields
        char lowThresholdStr[8];
        char highThresholdStr[8];
        sprintf(lowThresholdStr, "%d", settings.lowThreshold);
        sprintf(highThresholdStr, "%d", settings.highThreshold);
        
        // Low Threshold Input (below this is RED)
        WiFiManagerParameter custom_low_threshold("lowThreshold", "Low Threshold (RED) mg/dL", lowThresholdStr, 5);
        wifiManager.addParameter(&custom_low_threshold);
        
        // High Threshold Input (above this is YELLOW, between is GREEN)
        WiFiManagerParameter custom_high_threshold("highThreshold", "High Threshold (YELLOW) mg/dL", highThresholdStr, 5);
        wifiManager.addParameter(&custom_high_threshold);
        
        // Add description for critical values flashing feature
        WiFiManagerParameter custom_html_critical_info("<p style='font-size: 0.8em; color: #888;'>Critical values will flash slowly when reached (0 = disabled)</p>");
        wifiManager.addParameter(&custom_html_critical_info);
        
        // Critical Low Value Input
        char criticalLowValueStr[8];
        sprintf(criticalLowValueStr, "%d", settings.criticalLowGlucoseValue);
        WiFiManagerParameter custom_critical_low_value("criticalLowGlucoseValue", "Critical Low Value (0=disable)", criticalLowValueStr, 5);
        wifiManager.addParameter(&custom_critical_low_value);
        
        // Critical High Value Input
        char criticalHighValueStr[8];
        sprintf(criticalHighValueStr, "%d", settings.criticalHighGlucoseValue);
        WiFiManagerParameter custom_critical_high_value("criticalHighGlucoseValue", "Critical High Value (0=disable)", criticalHighValueStr, 5);
        wifiManager.addParameter(&custom_critical_high_value);
        // --- End Glucose Threshold Parameters ---
        
        // Set portal options to ensure buttons are displayed
        wifiManager.setSaveConfigCallback([]() {
            Serial.println("Configuration saved");
        });
        
        // Set portal title
        wifiManager.setTitle("SugrBee Setup");
        
        // Don't hide the save button but do set it to auto-connect after save to prevent hanging
        wifiManager.setSaveConnect(true);  // Changed to true to auto-connect after save

        // Set a shorter timeout for the config portal
        wifiManager.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT_SEC);
        wifiManager.setConnectTimeout(30);
        
        // Reset settings if needed
        if (shouldResetSettings) {
            wifiManager.resetSettings();
        }
        
        // Connect to WiFi or start config portal
        if (!wifiManager.autoConnect("SugrBee Setup")) { // Explicitly set AP name here
            Serial.println("Failed to connect and hit timeout");
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(20, 60);
            tft.println("Connection failed!");
            tft.setCursor(20, 90);
            tft.println("Restarting...");
            delay(3000);
            return false;
        }
        
        // Connected to WiFi - save this network for multi-WiFi reconnection
        {
            String connectedSSID = WiFi.SSID();
            String connectedPSK = WiFi.psk();
            if (connectedSSID.length() > 0) {
                settings.addWifiNetwork(connectedSSID.c_str(), connectedPSK.c_str());
                Serial.printf("Saved WiFi network: %s\n", connectedSSID.c_str());
            }
            // Populate WiFiMulti with all stored networks
            for (int i = 0; i < settings.wifiNetworkCount; i++) {
                wifiMulti.addAP(settings.wifiNetworks[i].ssid, settings.wifiNetworks[i].password);
            }
        }
        
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(2);
        tft.setCursor(20, 60);
        tft.println("WiFi Connected!");
        tft.setCursor(20, 90);
        tft.println("IP: " + WiFi.localIP().toString());
        
        // Wait a bit after connection
        delay(2000);
        
        // Save parameters
        // Device name (optional - shown at top of main screen)
        strncpy(settings.deviceName, custom_device_name.getValue(), sizeof(settings.deviceName) - 1);
        settings.deviceName[sizeof(settings.deviceName) - 1] = '\0';
        
        strncpy(settings.dexcomUsername, custom_dexcom_username.getValue(), sizeof(settings.dexcomUsername) - 1);
        settings.dexcomUsername[sizeof(settings.dexcomUsername) - 1] = '\0';
        
        strncpy(settings.dexcomPassword, custom_dexcom_password.getValue(), sizeof(settings.dexcomPassword) - 1);
        settings.dexcomPassword[sizeof(settings.dexcomPassword) - 1] = '\0';
        
        // Get server region from the tracked hidden field (kept in sync with the visible
        // dropdown by its onchange handler - see custom_server_select above).
        const char* regionValue = custom_server_region.getValue();
        if (regionValue && strlen(regionValue) > 0) {
            int regionVal = atoi(regionValue);
            settings.serverRegion = (regionVal >= 0 && regionVal <= 2) ? regionVal : 0;
        }
        
        // Save LibreView parameters
        strncpy(settings.libreViewEmail, custom_libreview_email.getValue(), sizeof(settings.libreViewEmail) - 1);
        settings.libreViewEmail[sizeof(settings.libreViewEmail) - 1] = '\0';
        
        strncpy(settings.libreViewPassword, custom_libreview_password.getValue(), sizeof(settings.libreViewPassword) - 1);
        settings.libreViewPassword[sizeof(settings.libreViewPassword) - 1] = '\0';
        
        // LibreView region is auto-detected from API redirect - no manual save needed
        
        // Save glucose threshold settings - New 3-color scheme: LOW=RED, GOOD=GREEN, HIGH=YELLOW
        int newLowThreshold = atoi(custom_low_threshold.getValue());
        int newHighThreshold = atoi(custom_high_threshold.getValue());
        
        // Validate thresholds (make sure they're in a reasonable range and order)
        if (newLowThreshold >= 40 && newLowThreshold <= 120 && 
            newHighThreshold >= 150 && newHighThreshold <= 350 && 
            newLowThreshold < newHighThreshold) {
            settings.lowThreshold = newLowThreshold;
            settings.highThreshold = newHighThreshold;
            // Note: normalThreshold is no longer used in the 3-color scheme
        }
        
        // Save critical glucose values for flashing feature (0 = disabled)
        // No need for range validation as any valid glucose reading is acceptable
        int newCriticalLowValue = atoi(custom_critical_low_value.getValue());
        int newCriticalHighValue = atoi(custom_critical_high_value.getValue());
        settings.criticalLowGlucoseValue = newCriticalLowValue;
        settings.criticalHighGlucoseValue = newCriticalHighValue;
        
        // Save settings
        settings.save();
        
        return true;
    }

    void checkConnection() {
        if (WiFi.status() == WL_CONNECTED) {
            return;
        }

        Serial.println("WiFi connection lost - attempting to reconnect...");
        Serial.printf("Stored WiFi networks: %d\n", settings.wifiNetworkCount);

        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setCursor(20, 60);
        tft.println("WiFi connection lost");
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(20, 90);
        tft.println("Reconnecting...");

        WiFi.disconnect();
        delay(100);
        
        bool reconnected = false;
        
        // Try multi-WiFi reconnection if we have stored networks
        if (settings.wifiNetworkCount > 1) {
            Serial.println("Trying all stored WiFi networks...");
            tft.setCursor(20, 120);
            tft.setTextSize(1);
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.printf("Scanning %d networks...", settings.wifiNetworkCount);
            
            // wifiMulti.run() scans and connects to the best available network
            if (wifiMulti.run(15000) == WL_CONNECTED) { // 15 second timeout
                reconnected = true;
                Serial.printf("Connected to: %s\n", WiFi.SSID().c_str());
            }
        }
        
        // Fallback: try last connected network directly
        if (!reconnected) {
            WiFi.begin(); // Reuse last stored credentials
            const unsigned long reconnectTimeoutMs = 10000; // 10 seconds
            unsigned long startAttempt = millis();
            while (millis() - startAttempt < reconnectTimeoutMs) {
                if (WiFi.status() == WL_CONNECTED) {
                    reconnected = true;
                    break;
                }
                delay(250);
            }
        }

        if (reconnected) {
            Serial.printf("Reconnected to WiFi: %s\n", WiFi.SSID().c_str());
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setCursor(40, 80);
            tft.println("WiFi Reconnected!");
            delay(1500);
            displayManager.clearDiagnosticState();
        } else {
            Serial.println("WiFi reconnect attempt failed");
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.setCursor(40, 130);
            tft.println("Still disconnected");
            tft.setCursor(40, 160);
            tft.println("Will retry soon...");
            delay(1500);
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_WIFI_DISCONNECTED_PUBLIC);
        }

        // Ensure the main UI redraws so we don't get stuck on the reconnect screen
        displayManager.forceFullUpdate();
    }
};

// Global WiFi helper
WifiHelper wifiHelper;

// Check if it's time for an update considering millis() rollover
bool checkTimeForUpdate(unsigned long currentTime, unsigned long lastUpdate, unsigned long interval) {
    // Check if millis() has rolled over
    if (currentTime < lastUpdate) {
        // Calculate time elapsed considering the rollover
        return ((0xFFFFFFFF - lastUpdate) + currentTime) >= interval;
    } else {
        // Normal case, no rollover
        return (currentTime - lastUpdate) >= interval;
    }
}

// Setup time via NTP
void setupTime() {
    // Use the time zone offset from settings
    const int gmtOffset_sec = settings.timeZoneOffset * 3600;  // Convert hours to seconds
    const int daylightOffset_sec = 0;     // Already included in gmtOffset_sec
    
    // Force time refresh when time zone changes
    static int lastTimeZoneOffset = 9999; // Initialize to impossible value
    bool timeZoneChanged = (lastTimeZoneOffset != settings.timeZoneOffset);
    
    if (timeZoneChanged) {
        Serial.print("Time zone changed from UTC");
        if (lastTimeZoneOffset >= 0 && lastTimeZoneOffset != 9999) Serial.print("+");
        if (lastTimeZoneOffset != 9999) Serial.print(lastTimeZoneOffset);
        Serial.print(" to UTC");
        if (settings.timeZoneOffset >= 0) Serial.print("+");
        Serial.println(settings.timeZoneOffset);
        
        lastTimeZoneOffset = settings.timeZoneOffset;
    }
    
    // Configure SNTP exactly once. Calling configTime() repeatedly (as the old code did
    // on a timezone change) restarts the SNTP client and can prevent the first sync from
    // ever completing - a major cause of boots stuck at the 1970 epoch ("12/31/69").
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov", "time.google.com");
    Serial.println("Waiting for NTP time sync...");
    Serial.print("Using time zone offset: UTC");
    if (settings.timeZoneOffset >= 0) {
        Serial.print("+");
    }
    Serial.println(settings.timeZoneOffset);
    
    // Wait up to ~20 seconds for the first sync. NTP uses UDP/123, which can take longer
    // than a few seconds to complete on some networks, especially right after boot.
    int retryCount = 0;
    time_t now = time(nullptr);
    while (now < 24 * 3600 && retryCount < 40) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
        retryCount++;
    }
    Serial.println();
    
    // Log the synchronized time
    if (now > 24 * 3600) {
        struct tm timeInfo;
        localtime_r(&now, &timeInfo);
        
        char strftime_buf[64];
        int hour = timeInfo.tm_hour % 12;
        if (hour == 0) hour = 12; // Convert 0 to 12 for midnight/noon
        
        const char* dayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
        const char* dayName = dayNames[timeInfo.tm_wday];
        
        // Using hour variable already declared above

        const char* ampm = (timeInfo.tm_hour >= 12) ? "PM" : "AM";
        
        snprintf(strftime_buf, sizeof(strftime_buf), "%d:%02d%s %s %02d/%02d/%02d", 
                 hour, timeInfo.tm_min, ampm, 
                 dayName,
                 timeInfo.tm_mon + 1, timeInfo.tm_mday, 
                 (timeInfo.tm_year + 1900) % 100);
        
        Serial.print("Synchronized time: ");
        Serial.println(strftime_buf);
    } else {
        Serial.println("Failed to sync time, using default");
    }
    
    // After setting time, force display update
    if (displayManager.isInitialized()) {
        displayManager.setTimeNeedsUpdate(true);
        displayManager.updateTimeDisplay();
    }
    
    // If this was a time zone change, force another time update after a short delay
    if (timeZoneChanged) {
        delay(100);
        now = time(nullptr);
        struct tm timeInfo;
        localtime_r(&now, &timeInfo);
        
        char timeStr[30];
        int hour = timeInfo.tm_hour % 12;
        if (hour == 0) hour = 12;
        
        const char* dayNames[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
        const char* dayName = dayNames[timeInfo.tm_wday];
        
        const char* ampm = (timeInfo.tm_hour >= 12) ? "PM" : "AM";
        
        snprintf(timeStr, sizeof(timeStr), "%d:%02d%s %s %02d/%02d/%02d", 
                 hour, timeInfo.tm_min, ampm, 
                 dayName,
                 timeInfo.tm_mon + 1, timeInfo.tm_mday, 
                 (timeInfo.tm_year + 1900) % 100);
        
        Serial.print("Current time after update: ");
        Serial.println(timeStr);
        
        // Get and display current time after update
        displayManager.setTimeNeedsUpdate(true);
        displayManager.updateTimeDisplay();
    }
}

// Display firmware version in small font at bottom right
void displayFirmwareVersion() {
    tft.setTextSize(1); // Small font
    tft.setTextColor(TFT_DARKGREY);
    tft.setCursor(tft.width() - 60, tft.height() - 10); // Bottom right
    tft.print(FIRMWARE_VERSION);
}

// Write embedded PNG arrow assets to SPIFFS if any are missing
void writeEmbeddedPNGs() {
    int written = 0;
    for (int i = 0; i < EMBEDDED_PNG_COUNT; i++) {
        if (!SPIFFS.exists(EMBEDDED_PNGS[i].filename)) {
            File f = SPIFFS.open(EMBEDDED_PNGS[i].filename, FILE_WRITE);
            if (f) {
                // Read from PROGMEM into a temp buffer and write to SPIFFS
                uint8_t buf[128];
                unsigned int remaining = EMBEDDED_PNGS[i].length;
                unsigned int offset = 0;
                while (remaining > 0) {
                    unsigned int chunk = (remaining < sizeof(buf)) ? remaining : sizeof(buf);
                    memcpy_P(buf, EMBEDDED_PNGS[i].data + offset, chunk);
                    f.write(buf, chunk);
                    offset += chunk;
                    remaining -= chunk;
                }
                f.close();
                written++;
                Serial.printf("Wrote embedded PNG: %s (%u bytes)\n", EMBEDDED_PNGS[i].filename, EMBEDDED_PNGS[i].length);
            } else {
                Serial.printf("Failed to write PNG: %s\n", EMBEDDED_PNGS[i].filename);
            }
        }
    }
    if (written > 0) {
        Serial.printf("Wrote %d missing PNG files to SPIFFS\n", written);
    } else {
        Serial.println("All PNG arrow assets present in SPIFFS");
    }
}

// Main setup function
void setup() {
    Serial.begin(115200);
    Serial.println("\nDexcom Monitor Starting...");
    Serial.print("Free heap at start: ");
    Serial.println(ESP.getFreeHeap());
    
    // Record boot time for grace period calculation
    g_bootTime = millis();
    
    // Initialize display
    tft.init();
    tft.setRotation(1); // Landscape mode
    
    // Apply gamma correction for improved display colors
    tft.writecommand(ILI9341_GAMMASET); //Gamma curve selected
    tft.writedata(2);
    delay(120);
    tft.writecommand(ILI9341_GAMMASET); //Gamma curve selected
    tft.writedata(1);
    
    // Fix inverted colors
    tft.invertDisplay(false);
    
    // Set black background and white text
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // Initialize PWM for backlight control (ESP32 Arduino 3.x)
    ledcAttach(LCD_BACK_LIGHT_PIN, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    
    // Initialize SPIFFS for storing settings FIRST (before showing startup screen)
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed - Formatting");
        if (!SPIFFS.format()) {
            Serial.println("SPIFFS format failed");
        } else {
            Serial.println("SPIFFS formatted successfully");
            if (!SPIFFS.begin()) {
                Serial.println("SPIFFS mount still failed after format");
            } else {
                Serial.println("SPIFFS mounted after format");
            }
        }
    } else {
        Serial.println("SPIFFS mounted successfully");
    }
    
    // Write any missing embedded PNG arrow assets to SPIFFS
    writeEmbeddedPNGs();
    
    // Load saved settings BEFORE displaying anything
    settings.load();
    
    // Populate WiFiMulti with all stored networks for reconnection
    for (int i = 0; i < settings.wifiNetworkCount; i++) {
        wifiMulti.addAP(settings.wifiNetworks[i].ssid, settings.wifiNetworks[i].password);
        Serial.printf("WiFiMulti: added stored network '%s'\n", settings.wifiNetworks[i].ssid);
    }
    
    // Apply saved brightness setting
    setBrightness(settings.brightness);
    Serial.printf("Applied saved brightness: %d (%d%%)\n", settings.brightness, (settings.brightness * 100) / 255);
    
    // Apply saved rotation setting BEFORE showing startup screen
    if (settings.isDisplayRotated) {
        tft.setRotation(3);  // 180 degree rotation
        Serial.println("Applied saved rotation: 180 degrees");
    } else {
        tft.setRotation(1);  // Normal landscape
        Serial.println("Applied saved rotation: Normal");
    }
    
    // Initialize display manager and show startup screen (now with correct rotation)
    displayManager.init();
    displayManager.setRotationState(settings.isDisplayRotated);
    displayManager.showInitialScreen();
    
    // Initialize touch screen (after rotation is set)
    touchHandler.init();
    
    // Apply rotation to touchscreen
    if (settings.isDisplayRotated) {
        touchscreen.setRotation(3);
    } else {
        touchscreen.setRotation(1);
    }
    
    // Set reset pin as input
    pinMode(RESET_PIN, INPUT_PULLUP);
    
    // Check if reset button is pressed during boot
    if (digitalRead(RESET_PIN) == LOW) {
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(20, 60);
        tft.println("Reset button pressed");
        tft.setCursor(20, 90);
        tft.println("Resetting settings...");
        shouldResetSettings = true;
        delay(2000);
    }
    
    // Connect to WiFi
    if (!wifiHelper.connect()) {
        ESP.restart();
    }
    
    // Disable WiFi power-saving mode to prevent auto-sleep
    WiFi.setSleep(false);
    
    // Disable automatic light sleep mode
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    // Wait for WiFi to fully stabilize before any network requests (NTP + timezone).
    // Previously NTP was attempted immediately after association - before DHCP/routing
    // had settled - which was a frequent cause of failed time sync at boot.
    tft.setCursor(20, 120);
    tft.println("Stabilizing WiFi...");
    delay(3000); // Give WiFi time to fully stabilize
    
    // Verify WiFi is still connected and test DNS connectivity
    bool wifiReady = false;
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Testing WiFi connectivity before timezone detection...");
        Serial.printf("WiFi Signal: %d dBm\n", WiFi.RSSI());
        Serial.printf("Local IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
        
        // Test DNS resolution before attempting timezone fetch
        IPAddress resolved;
        Serial.println("Testing DNS resolution (google.com)...");
        unsigned long dnsStart = millis();
        while ((millis() - dnsStart) < 5000) {
            if (WiFi.hostByName("google.com", resolved)) {
                Serial.printf("DNS working! Resolved to %s in %lums\n", resolved.toString().c_str(), millis() - dnsStart);
                wifiReady = true;
                break;
            }
            delay(500);
        }
        if (!wifiReady) {
            Serial.println("DNS resolution failed after 5 seconds");
        }
    } else {
        Serial.println("WiFi not ready for timezone detection");
    }
    
    // Configure time via NTP now that the network path is ready (DHCP/DNS settled).
    tft.fillRect(20, 120, 300, 40, TFT_BLACK); // Clear stabilizing/status lines
    tft.setCursor(20, 120);
    tft.println("Setting up time...");
    setupTime();
    
    // Now detect the timezone (corrects the saved offset if it has changed / DST flipped)
    tft.setCursor(20, 160);
    tft.println("Detecting timezone...");
    
    if (wifiReady && fetchAutoTimezone()) {
        Serial.println("Automatic timezone detection successful");
    } else {
        Serial.printf("Timezone detection failed, using saved timezone (UTC%+d)\n", settings.timeZoneOffset);
    }
    tft.fillRect(20, 160, 280, 20, TFT_BLACK); // Clear "Detecting timezone..."
    tft.setCursor(20, 160);
    tft.println("Timezone detected");
    delay(1000);
    
    // Show disclaimer screen if not yet accepted
    if (!settings.disclaimerAccepted) {
        Serial.println("Showing disclaimer screen...");
        displayManager.showDisclaimerScreen();
        
        // Wait for user to touch "I Agree" button
        bool disclaimerAccepted = false;
        while (!disclaimerAccepted) {
            if (touchscreen.tirqTouched() && touchscreen.touched()) {
                TS_Point p = touchscreen.getPoint();
                // Map touch coordinates to screen (must match main handleTouch mapping)
                // Note: touchscreen.setRotation() already handles rotation, no manual inversion needed
                int touchX = map(p.x, 200, 3700, 0, 320);
                int touchY = map(p.y, 240, 3800, 0, 240);
                
                Serial.print("Disclaimer touch at X=");
                Serial.print(touchX);
                Serial.print(", Y=");
                Serial.println(touchY);
                
                if (displayManager.isDisclaimerButtonPressed(touchX, touchY)) {
                    settings.disclaimerAccepted = true;
                    settings.save();
                    disclaimerAccepted = true;
                    Serial.println("Disclaimer accepted");
                }
                delay(200); // Debounce
            }
            delay(50); // Small delay to avoid busy waiting
        }
    }
    
    // Initialize the active glucose provider (LibreView or Dexcom) and authenticate.
    // Provider is auto-detected from which credentials were configured during setup.
    bool libreMode = isLibreUser();
    bool authOk = false;
    if (libreMode) {
        Serial.println("Boot: LibreView credentials detected - using LibreView as glucose source");
        libreClient.init();
        authOk = libreClient.authenticate();
        if (!authOk) {
            Serial.println("ERROR: LibreView authentication failed on boot");
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setCursor(20, 60);
            tft.println("LibreView login failed");
            tft.setTextSize(1);
            tft.setCursor(20, 95);
            tft.println("Will retry automatically...");
        }
    } else {
        // Initialize Dexcom client
        dexcomClient.init();
        
        // Verify credentials
        if (strlen(settings.dexcomUsername) == 0 || strlen(settings.dexcomPassword) == 0) {
            Serial.println("ERROR: Dexcom credentials are empty!");
            
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(20, 60);
            tft.println("Missing Dexcom credentials");
            tft.setCursor(20, 90);
            tft.println("Reset & reconfigure");
            return;
        }
        
        // Connect to Dexcom (no on-screen splash to avoid confusing stale-data UX)
        // Try authentication with retry
        authOk = dexcomClient.authenticateWithRetry() && dexcomClient.login();
    }
    
    if (authOk) {
        {
            // Initialize variables
            float current_glucose = 0;
            float previous_glucose = 0;
            char trend[20] = "";
            char timestamp[10] = "";
            unsigned long long dexcomTime = 0;
            
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setCursor(10, 60);  // Move further left
            tft.println("Getting latest");
            tft.setCursor(10, 90);  // Position for second line
            tft.println("readings...");
            
            // Get initial glucose data from the active provider
            bool gotInitialData = libreMode
                ? libreClient.fetchGlucoseData(current_glucose, previous_glucose, trend, timestamp, dexcomTime)
                : dexcomClient.fetchGlucoseData(current_glucose, previous_glucose, trend, timestamp, dexcomTime);
            if (gotInitialData && libreMode) {
                // LibreView's graphData previous is a lagging 15-min average, so it makes a
                // misleading boot differential. Seed the reading-to-reading tracker and show a
                // 0 delta on the first reading; subsequent readings compute a true delta.
                previous_glucose = current_glucose;
                g_librePreviousGlucose = current_glucose;
            }
            if (gotInitialData) {
                // Initialize the static variables that will be used in the main loop
                unsigned long currentTime = millis();
                g_lastUpdateTime = currentTime;
                g_lastDexcomReadingTime = dexcomTime;
                g_lastSuccessfulFetchTime = currentTime;  // Track successful fetch for grace period
                
                // Check if the reading is actually fresh before displaying it
                time_t bootNow = time(nullptr);
                unsigned long long bootNowMs = ((unsigned long long)bootNow) * 1000ULL;
                unsigned long bootReadingAgeSec = (bootNowMs > dexcomTime) ? (unsigned long)((bootNowMs - dexcomTime) / 1000ULL) : 0;
                
                if (bootReadingAgeSec < DisplayManager::STALE_AFTER_SECONDS_PUBLIC) {
                    // Reading is fresh - show it normally and start the post-reading grace window.
                    g_lastNewReadingReceivedTime = currentTime;  // a genuinely fresh reading
                    displayManager.clearDiagnosticState();
                    displayManager.setGlucoseData(current_glucose, previous_glucose, trend, timestamp, dexcomTime);
                } else {
                    // Reading is already stale at boot (e.g. a sensor gap before power-on). Still store
                    // the value so the status bar can show "(was xxx)", but force the "No Data" state so
                    // the big number isn't shown as if it were current. Do NOT start the post-reading
                    // grace window - this is not a fresh reading, and doing so would suppress "No Data"
                    // for 60s and make the main display show dashes instead.
                    Serial.printf("Boot: reading is %lu seconds old (stale) - showing No Data (was %.0f)\n",
                                  bootReadingAgeSec, current_glucose);
                    g_lastNewReadingReceivedTime = 0;
                    displayManager.setGlucoseData(current_glucose, previous_glucose, trend, timestamp, dexcomTime);
                    displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_DATA_STALE_PUBLIC);
                }
                {
                    time_t fetchNow = time(nullptr);
                    unsigned long long fetchEpochMillis = ((unsigned long long)fetchNow) * 1000ULL;
                    unsigned long ageAtFetchSec = (fetchEpochMillis > dexcomTime)
                                                      ? (unsigned long)((fetchEpochMillis - dexcomTime) / 1000ULL)
                                                      : 0;
                    g_lastReadingAgeAtFetchSec = (ageAtFetchSec <= FRESH_READING_MAX_AGE_SEC) ? 0 : ageAtFetchSec;
                }
                if (dexcomTime > 0) {
                    // Next boundary is one reading-interval (+5s buffer) after this reading
                    unsigned long long interval = (unsigned long long)readingIntervalMs();
                    unsigned long long boundaryBase = (dexcomTime / interval) + 1ULL;
                    g_nextReadingBoundaryEpochMs = (boundaryBase * interval) + 5000ULL;
                }
                
                // Update glucose history on startup
                int count = 0;
                bool gotHistory = libreMode
                    ? libreClient.fetchGlucoseHistory(glucoseHistory, count)
                    : dexcomClient.fetchGlucoseHistory(glucoseHistory, count);
                if (gotHistory) {
                    glucoseHistoryCount = count;
                    g_lastHistoryFetchTime = currentTime;  // Track history fetch time
                    Serial.printf("Loaded %d glucose history readings on startup\n", count);
                }
                
                // Clear screen before showing the actual glucose data display
                tft.fillScreen(TFT_BLACK);
                displayManager.updateDisplay();
                
                Serial.println("Initial glucose data loaded and displayed successfully");
            } else {
                // No glucose data available on boot - still show the display with "No data yet"
                Serial.println("No initial glucose data available, showing default display");
                
                // Clear screen and show the default display state
                tft.fillScreen(TFT_BLACK);
                displayManager.updateDisplay();
                
                // Ensure time display shows immediately
                displayManager.updateTimeDisplay();
            }
        }
    }
    
    Serial.print("Free heap after setup: ");
    Serial.println(ESP.getFreeHeap());
    
    // Apply saved inversion setting or default
    tft.invertDisplay(settings.isDisplayInverted);
    
    // Check for updates silently on boot (after WiFi is connected)
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Checking for updates on boot...");
        displayManager.checkForUpdateSilent();
    }
}

// Main loop function
void loop() {
    unsigned long currentTime = millis();
    
    // Handle touch input - HIGHEST PRIORITY for responsiveness
    // Process touch events before anything else, especially during glucose fetching
    static unsigned long lastTouchCheck = 0;
    if (touchHandler.isTouchEnabled() && (currentTime - lastTouchCheck >= 25)) { // Check touch every 25ms for better responsiveness
        lastTouchCheck = currentTime;
        bool shouldRestart = touchHandler.handleTouch(currentTime);
        
        // If reset credentials was confirmed, restart device
        if (shouldRestart) {
            ESP.restart();
        }
        
        // If WiFi button was tapped, open captive portal to add a network
        if (displayManager.shouldLaunchWifiPortal()) {
            displayManager.clearWifiPortalFlag();
            displayManager.toggleSettings(false);
            
            tft.fillScreen(TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.setCursor(30, 40);
            tft.println("Starting WiFi");
            tft.setCursor(30, 65);
            tft.println("Setup Portal...");
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setCursor(30, 100);
            tft.println("Connect to WiFi network:");
            tft.setTextSize(2);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setCursor(30, 120);
            tft.println("SugrBee Setup");
            tft.setTextSize(1);
            tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
            tft.setCursor(30, 150);
            tft.println("Then open 192.168.4.1");
            tft.setCursor(30, 165);
            tft.println("to select a WiFi network.");
            tft.setCursor(30, 190);
            tft.println("Network may take up to");
            tft.setCursor(30, 205);
            tft.println("2 minutes to appear.");
            
            // Fully tear down STA connection before starting AP
            Serial.println("WiFi portal: tearing down STA connection...");
            WiFi.disconnect(false); // Disconnect but don't erase NVS (we manage our own creds)
            
            // Wait for actual disconnection (up to 3 seconds)
            unsigned long disconnectStart = millis();
            while (WiFi.status() == WL_CONNECTED && (millis() - disconnectStart) < 3000) {
                delay(100);
            }
            Serial.printf("WiFi portal: disconnect took %lums, status=%d\n", 
                          millis() - disconnectStart, WiFi.status());
            
            // Switch to AP mode and let it stabilize
            WiFi.mode(WIFI_OFF);
            delay(500);
            WiFi.mode(WIFI_AP);
            delay(1000); // Let AP radio initialize
            Serial.println("WiFi portal: AP mode ready");
            
            WiFiManager wifiPortal;
            wifiPortal.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT_SEC);
            wifiPortal.setConnectTimeout(15);       // 15 sec to try connecting to chosen network
            // Exit the portal after the user clicks Save, even if no WiFi network was selected.
            // This lets users update just the device name without picking a WiFi - otherwise the
            // portal would keep running until the 1:30 timeout, leaving the device stuck on the
            // "Starting WiFi Setup Portal..." screen.
            wifiPortal.setBreakAfterConfig(true);
            
            // Optional device name shown at the top of the main screen (same field as initial setup).
            // Lets users edit the name without doing a full factory reset.
            WiFiManagerParameter custom_html_devicename_label("<p style='padding-top: 5px; padding-bottom: 0px;'><b>Device Name (optional)</b><br><span style='font-size:0.8em;color:#888;'>Shown at top of screen - leave blank to hide</span></p>");
            WiFiManagerParameter custom_device_name("deviceName", "Device Name", settings.deviceName, 23);
            wifiPortal.addParameter(&custom_html_devicename_label);
            wifiPortal.addParameter(&custom_device_name);
            wifiPortal.setTitle("SugrBee Setup");
            
            Serial.println("WiFi portal: starting config portal...");
            bool portalConnected = wifiPortal.startConfigPortal("SugrBee Setup");
            
            // Save device name regardless of WiFi connect outcome (the form submission persists
            // the value into the parameter object even if the user later cancels WiFi).
            const char* submittedName = custom_device_name.getValue();
            if (submittedName != nullptr && strcmp(submittedName, settings.deviceName) != 0) {
                strncpy(settings.deviceName, submittedName, sizeof(settings.deviceName) - 1);
                settings.deviceName[sizeof(settings.deviceName) - 1] = '\0';
                settings.save();
                Serial.printf("WiFi portal: device name updated to '%s'\n", settings.deviceName);
                // Force a full redraw so the new name appears immediately at the top of the screen
                displayManager.forceFullUpdate();
            }
            
            if (portalConnected) {
                // Connected - save the new network
                String newSSID = WiFi.SSID();
                String newPSK = WiFi.psk();
                Serial.printf("WiFi portal: connected to '%s'\n", newSSID.c_str());
                if (newSSID.length() > 0) {
                    settings.addWifiNetwork(newSSID.c_str(), newPSK.c_str());
                    wifiMulti.addAP(newSSID.c_str(), newPSK.c_str());
                    Serial.printf("Added WiFi network: %s (%d/%d stored)\n", 
                                  newSSID.c_str(), settings.wifiNetworkCount, MAX_WIFI_NETWORKS);
                }
                tft.fillScreen(TFT_BLACK);
                tft.setTextSize(2);
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.setCursor(30, 80);
                tft.println("WiFi Added!");
                delay(1500);
            } else {
                // Timed out or user didn't connect - reconnect to a stored network
                Serial.println("WiFi portal: timed out or cancelled");
                tft.fillScreen(TFT_BLACK);
                tft.setTextSize(2);
                tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                tft.setCursor(30, 80);
                tft.println("Portal closed");
                tft.setTextSize(1);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.setCursor(30, 110);
                tft.println("Reconnecting...");
                delay(500);
            }
            
            // Always reconnect to stored networks after portal closes
            WiFi.mode(WIFI_STA);
            delay(500);
            // Re-populate wifiMulti with all stored networks (portal may have cleared them)
            for (int i = 0; i < settings.wifiNetworkCount; i++) {
                wifiMulti.addAP(settings.wifiNetworks[i].ssid, settings.wifiNetworks[i].password);
            }
            
            // Try to connect with timeout
            Serial.println("WiFi portal: reconnecting to stored networks...");
            unsigned long reconnectStart = millis();
            while (wifiMulti.run() != WL_CONNECTED && (millis() - reconnectStart) < 15000) {
                delay(500);
                Serial.print(".");
            }
            Serial.println();
            
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("WiFi portal: reconnected to %s\n", WiFi.SSID().c_str());
            } else {
                Serial.println("WiFi portal: could not reconnect - will retry in main loop");
            }
            
            // Force full display refresh
            displayManager.forceFullUpdate();
        }
    }
    
    // Check for serial debug commands
    checkSerialDebugCommands();
    
    // Periodically check critical glucose status to ensure flashing works correctly
    static unsigned long lastCriticalCheck = 0;
    if (currentTime - lastCriticalCheck >= 10000) { // Check every 10 seconds
        lastCriticalCheck = currentTime;
        displayManager.checkCriticalGlucose(); // Re-check if current glucose is critical
    }
    
    // Handle glucose value flashing if needed
    displayManager.checkAndToggleFlash(currentTime);
    
    // Check for updates at midnight (once per day)
    static int lastUpdateCheckDay = -1;
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    if (timeinfo != nullptr && timeinfo->tm_year > 100) { // Valid time (after year 2000)
        int currentDay = timeinfo->tm_yday;
        int currentHour = timeinfo->tm_hour;
        int currentMinute = timeinfo->tm_min;
        
        // Check at midnight (00:00 - 00:05 window to avoid missing it)
        if (currentHour == 0 && currentMinute < 5 && currentDay != lastUpdateCheckDay) {
            lastUpdateCheckDay = currentDay;
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("Midnight update check triggered");
                displayManager.checkForUpdateSilent();
            }
        }
    }
    
    // WiFi reconnection guardrail: detect reconnect and immediately recover
    static wl_status_t lastWiFiStatus = WL_IDLE_STATUS;
    wl_status_t currentWiFiStatus = WiFi.status();
    if (currentWiFiStatus != lastWiFiStatus) {
        Serial.printf("WiFi status changed: %d -> %d\n", lastWiFiStatus, currentWiFiStatus);
        // On reconnect, re-auth/login if needed and force a fetch + display update
        if (currentWiFiStatus == WL_CONNECTED) {
            Serial.println("WiFi reconnected - initiating glucose recovery fetch");
            // Clear stale latch so UI can update immediately
            g_hasShownStaleMessage = false;
            // If session is missing, re-establish it.
            // LibreView handles its own auth inside updateLibreData(), so only run the
            // Dexcom re-auth path for Dexcom users (avoids "credentials empty" errors).
            bool ok = true;
            if (!isLibreUser() && strlen(dexcomClient.getSessionId()) == 0) {
                ok = dexcomClient.authenticateWithRetry(false) && dexcomClient.login(false);
            }
            if (ok) {
                updateDexcomData(currentTime, g_lastUpdateTime, g_lastDexcomReadingTime, false);
                // Force UI refresh regardless
                displayManager.clearDiagnosticState();
                displayManager.forceFullUpdate();
                displayManager.updateDisplay();
                displayManager.updateTimeDisplay();
            } else {
                // Could not re-establish session yet; mark WiFi ok but session expired
                displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_SESSION_EXPIRED_PUBLIC);
                displayManager.forceFullUpdate();
                displayManager.updateDisplay();
            }
        }
        lastWiFiStatus = currentWiFiStatus;
    }
    
    // Check if reset button is being held for 5 seconds
    if (digitalRead(RESET_PIN) == LOW) {
        unsigned long buttonPressStart = currentTime;
        while (digitalRead(RESET_PIN) == LOW && millis() - buttonPressStart < 5000) {
            delay(100);
        }
        
        // If button was held for 5 seconds
        if (millis() - buttonPressStart >= 5000) {
            tft.fillScreen(TFT_BLACK);
            tft.setCursor(20, 60);
            tft.println("Reset button pressed");
            tft.setCursor(20, 90);
            tft.println("Resetting settings...");
            shouldResetSettings = true;
            delay(2000);
            ESP.restart();
        }
    }
    
    // Check WiFi connection every 5 minutes
    static unsigned long lastWiFiCheckTime = 0;
    if (currentTime - lastWiFiCheckTime >= 300000) { // 5 minutes
        wifiHelper.checkConnection();
        lastWiFiCheckTime = currentTime;
    }
    
    // NTP recovery: if the clock never synced at boot (still at the 1970 epoch), keep
    // nudging SNTP in the background every 30s. This is non-blocking - configTime() just
    // (re)starts the async sync and a later loop iteration picks up the corrected time.
    static unsigned long lastNtpRetryTime = 0;
    if (time(nullptr) < 24 * 3600 && WiFi.status() == WL_CONNECTED) {
        if (lastNtpRetryTime == 0 || (currentTime - lastNtpRetryTime) >= 30000) {
            Serial.println("Clock not set - re-initializing NTP sync in background...");
            int gmtOffset_sec = settings.timeZoneOffset * 3600;
            configTime(gmtOffset_sec, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
            lastNtpRetryTime = currentTime;
        }
    }
    
    // Check timezone every 24 hours to handle DST changes and travel
    static unsigned long lastTimezoneCheckTime = 0;
    if (currentTime - lastTimezoneCheckTime >= 24 * 60 * 60 * 1000UL) { // 24 hours
        Serial.println("Performing daily timezone check...");
        if (fetchAutoTimezone()) {
            Serial.println("Daily timezone update successful");
        } else {
            Serial.println("Daily timezone update failed");
        }
        lastTimezoneCheckTime = currentTime;
    }
    
    // Dexcom data update check - this runs as needed based on last reading timestamp
    // Using global variables (already declared at the top of the file)
    
    // Check if we need an immediate fetch (e.g., after exiting settings)
    if (g_needsImmediateFetch) {
        Serial.println("Immediate fetch triggered (returning from settings)");
        g_needsImmediateFetch = false;
        updateDexcomData(currentTime, g_lastUpdateTime, g_lastDexcomReadingTime, false);
        g_lastUpdateTime = currentTime;
    }
    
    // If we have a previous reading time, use it to determine when to start checking
    if (g_lastDexcomReadingTime > 0) {
        // Convert to epoch time in milliseconds
        time_t now = time(nullptr);
        unsigned long long currentEpochMillis = ((unsigned long long)now) * 1000;
        
        // Calculate time since last reading in seconds
        unsigned long long timeSinceLastReading = currentEpochMillis - g_lastDexcomReadingTime;
        unsigned long secondsSinceLastReading = timeSinceLastReading / 1000;
        bool isStaleReading = (secondsSinceLastReading >= DisplayManager::STALE_AFTER_SECONDS_PUBLIC);
        
        // Skip glucose fetching while in settings screen to keep UI responsive
        bool inSettingsOrSubscreen = displayManager.isDisplayingSettings() || 
                                      displayManager.isDisplayingBrightnessConfig() ||
                                      displayManager.isDisplayingDownDetector();
        
        // Server outage detection:
        // - When data has been stale for >1 hour, poll the provider's status page every 15 min.
        // - If any monitored service is non-operational, set g_serverOutageDetected (shown in bottom bar).
        // - As soon as a fresh reading is received, clear the flag automatically.
        if (secondsSinceLastReading >= OUTAGE_STALE_THRESHOLD_SEC && !inSettingsOrSubscreen) {
            bool firstCheck = (g_lastOutageCheckTime == 0);
            if (firstCheck || (currentTime - g_lastOutageCheckTime) >= OUTAGE_CHECK_INTERVAL_MS) {
                Serial.printf("Data stale for %lus - checking %s status page\n",
                              secondsSinceLastReading, isLibreUser() ? "Libre" : "Dexcom");
                checkServerStatusForOutage();
            }
        } else if (secondsSinceLastReading < OUTAGE_STALE_THRESHOLD_SEC && g_serverOutageDetected && !g_debugForceOutage) {
            // Data flowing again - clear the warning and force a redraw
            // (Skip if debug override is active so testers can see the warning with fresh data.)
            Serial.println("Fresh data resumed - clearing server outage warning");
            g_serverOutageDetected = false;
            g_lastOutageCheckTime = 0; // reset so next stale episode starts fresh
            displayManager.forceFullUpdate();
        }
        
        // Start checking one reading-interval after the last reading (5:05 for Dexcom, 1:05 for Libre)
        // to give the provider time to process and publish the new value.
        if (secondsSinceLastReading >= proactiveStartSeconds() && !g_isWaitingForNewReading && !inSettingsOrSubscreen) {
            // If we have a next boundary and we're not stale yet, wait until boundary
            if (!isStaleReading && g_nextReadingBoundaryEpochMs > 0 &&
                currentEpochMillis < g_nextReadingBoundaryEpochMs) {
                // Skip proactive checks until the next boundary
            } else {
                Serial.printf("Starting proactive checks at %lu seconds since last reading\n", secondsSinceLastReading);
                updateDexcomData(currentTime, g_lastUpdateTime, g_lastDexcomReadingTime, false);
                g_lastUpdateTime = currentTime;
                g_isWaitingForNewReading = true; // Mark that we've started the checking cycle
                g_waitingStartTime = currentTime; // Track when we started waiting
                g_proactiveCheckCount = 1; // Count the initial proactive check
            }
        }
        
        // Ensure 'No Data' appears immediately at >= 6 minutes (no bypass due to early-return)
        if (isStaleReading) {
            if (!g_hasShownStaleMessage) {
                Serial.println("Crossed stale threshold - forcing diagnostic display");
                displayManager.forceFullUpdate();
                displayManager.updateDiagnosticState();
                displayManager.updateDisplay();
                g_hasShownStaleMessage = true;
            }
        } else if (g_hasShownStaleMessage) {
            // Reset the latch when we are below threshold again (e.g., after new reading)
            g_hasShownStaleMessage = false;
            g_stalePollCount = 0;
            g_nextReadingBoundaryEpochMs = 0;
        }

        // Staged checking strategy when waiting for new reading
        // Skip polling while in settings to keep UI responsive
        if (g_isWaitingForNewReading && !inSettingsOrSubscreen) {
            unsigned long checkInterval;
            
            // Check if we're in backoff mode due to 429 rate limiting
            bool inBackoff = (g_consecutive429Count > 0) && ((currentTime - g_last429Time) < 10000);
            
            if (inBackoff) {
                // Backoff mode: exponential backoff when rate limited (429)
                // Phase 1: 10 sec, Phase 2: 20 sec, Phase 3+: 30 sec
                if (g_consecutive429Count <= 1) {
                    checkInterval = 10000; // 10 seconds
                } else if (g_consecutive429Count == 2) {
                    checkInterval = 20000; // 20 seconds
                } else {
                    checkInterval = 30000; // 30 seconds
                }
            }
            else if (isStaleReading) {
                // Stale recovery: check every 10 seconds
                checkInterval = 10000;
            }
            else {
                // Normal polling: check every 10 seconds
                checkInterval = 10000;
            }

            // Boundary-aware lockout: don't poll again until next 5-min boundary unless stale
            if (!isStaleReading && g_nextReadingBoundaryEpochMs > 0 && currentEpochMillis < g_nextReadingBoundaryEpochMs) {
                unsigned long long timeUntilBoundary = g_nextReadingBoundaryEpochMs - currentEpochMillis;
                unsigned long boundaryInterval = (unsigned long)(timeUntilBoundary > 0 ? timeUntilBoundary : 0);
                if (boundaryInterval > checkInterval) {
                    checkInterval = boundaryInterval;
                }
            }

            // If we just fetched successfully but the reading didn't change, slow down
            if (g_lastNoNewReadingFetchTime > 0 &&
                (currentTime - g_lastNoNewReadingFetchTime) < NO_NEW_READING_COOLDOWN_MS) {
                if (checkInterval < NO_NEW_READING_COOLDOWN_MS) {
                    checkInterval = NO_NEW_READING_COOLDOWN_MS;
                }
            }
            
            if (currentTime - g_lastUpdateTime >= checkInterval) {
                Serial.printf("Checking at %lu seconds since last reading (interval: %lums%s)\n", 
                             secondsSinceLastReading, checkInterval, inBackoff ? " BACKOFF" : "");
                
                // Store the previous reading time for comparison
                unsigned long long previousDexcomReadingTime = g_lastDexcomReadingTime;
                
                // Clear the last HTTP error before fetching
                dexcomClient.clearLastHttpError();
                
                // Update Dexcom data
                updateDexcomData(currentTime, g_lastUpdateTime, g_lastDexcomReadingTime, secondsSinceLastReading > 325);
                g_lastUpdateTime = currentTime;
                g_proactiveCheckCount++;
                if (isStaleReading) {
                    g_stalePollCount++;
                }
                
                // Check for 429 rate limit error and update backoff state
                if (dexcomClient.getLastHttpError() == 429) {
                    g_consecutive429Count++;
                    g_last429Time = currentTime;
                    Serial.printf("Rate limited (429) - backoff count: %d\n", g_consecutive429Count);
                } else if (dexcomClient.getLastHttpError() == 0) {
                    // Successful fetch - reset backoff
                    g_consecutive429Count = 0;
                }
                
                // Check if we received a new reading by comparing timestamps directly
                if (g_lastDexcomReadingTime > previousDexcomReadingTime) {
                    Serial.println("New reading detected, stopping proactive checks");
                    g_isWaitingForNewReading = false;
                    g_consecutive429Count = 0;  // Reset backoff on success
                    g_proactiveCheckCount = 0;
                    g_stalePollCount = 0;
                } else if (!isStaleReading && secondsSinceLastReading < 290) {
                    // The proactive check that started this cycle already got the new reading,
                    // so g_lastDexcomReadingTime == previousDexcomReadingTime here even though
                    // a new reading was received. If we're well within the 5-min window, stop polling.
                    Serial.printf("Reading is fresh (%lus old) - stopping proactive checks\n", secondsSinceLastReading);
                    g_isWaitingForNewReading = false;
                    g_proactiveCheckCount = 0;
                }
            }
        }
        // Reset waiting state if we're far from the update window (helps recover from missed updates)
        else if (secondsSinceLastReading < 290 && g_isWaitingForNewReading) {
            g_isWaitingForNewReading = false;
            g_proactiveCheckCount = 0;
        }
    } 
    // If we don't have a Dexcom timestamp yet, need to fetch data
    // Skip while in settings to keep UI responsive
    else if (!displayManager.isDisplayingSettings() && !displayManager.isDisplayingBrightnessConfig() && !displayManager.isDisplayingDownDetector()) {
        // Retry more frequently (every 15 seconds) when we have no data at all
        // This handles the case where initial boot fetch failed
        unsigned long retryInterval = (g_lastDexcomReadingTime == 0) ? 15000 : UPDATE_INTERVAL;
        
        if (checkTimeForUpdate(currentTime, g_lastUpdateTime, retryInterval)) {
            if (g_lastDexcomReadingTime == 0) {
                Serial.println("No glucose data yet - retrying fetch...");
            }
            updateDexcomData(currentTime, g_lastUpdateTime, g_lastDexcomReadingTime, false);
            g_lastUpdateTime = currentTime;
        }
    }
    
    // Recompute the time/"X mins ago" display frequently to keep it accurate.
    static unsigned long lastTimeDisplayUpdate = 0;
    static unsigned long bootTime = 0;
    if (bootTime == 0) bootTime = currentTime; // Track boot time
    
    // Time display update strategy:
    // - First 2 min after boot: every 2 seconds (to ensure display initializes)
    // - After that: every 5 seconds, so the "X mins ago" counter never lags the real
    //   minute boundary by more than ~5s. updateTimeDisplay() skips the actual TFT redraw
    //   when the rendered text is unchanged, so this frequent recompute is essentially free
    //   and does not cause flicker.
    unsigned long timeUpdateInterval = (currentTime - bootTime < 120000) ? 2000 : 5000;
    
    if (currentTime - lastTimeDisplayUpdate >= timeUpdateInterval &&
        !displayManager.isDisplayingSettings() &&
        !displayManager.isDisplayingBrightnessConfig() &&
        !displayManager.isDisplayingDownDetector()) {
        lastTimeDisplayUpdate = currentTime;
        displayManager.updateTimeDisplay();
    }
    
    // Apply pending display updates
    if (displayManager.needsFullUpdate()) {
        if (displayManager.isDisplayingDownDetector()) {
            // Down Detector screen is drawn on-demand by the touch handler.
            // Don't redraw here - just keep the time display suppressed.
            displayManager.clearTimeUpdate();
        } else if (displayManager.isDisplayingBrightnessConfig()) {
            // Show brightness adjustment screen
            displayManager.drawBrightnessConfigScreen();
            displayManager.clearTimeUpdate();
        } else if (displayManager.isDisplayingSettings()) {
            // Only draw settings screen if no confirmation dialogs are active
            if (!displayManager.isDisplayingConfirmation() && !displayManager.isDisplayingUpdateConfirmation() && !displayManager.isDisplayingDownloadConfirmation()) {
                if (displayManager.getSettingsPage() == 2) {
                    displayManager.drawSettingsPage2();
                } else {
                    displayManager.drawSettingsScreen();
                }
            }
            
            // Show confirmation dialogs if active
            if (displayManager.isDisplayingConfirmation()) {
                displayManager.drawConfirmationDialog();
            } else if (displayManager.isDisplayingUpdateConfirmation()) {
                displayManager.drawUpdateConfirmationDialog();
            } else if (displayManager.isDisplayingDownloadConfirmation()) {
                displayManager.drawDownloadConfirmation();
            }
            
            // We never want to show the time on the settings screen
            displayManager.clearTimeUpdate();
        } else {
            displayManager.updateDisplay();
        }
        displayManager.clearFullUpdate();
        displayManager.clearTimeUpdate();
    } else if (displayManager.needsTimeUpdate() && 
              !displayManager.isDisplayingSettings()) {
        displayManager.updateTimeDisplay();
        displayManager.clearTimeUpdate();
    }
    
    delay(100); // Small delay to prevent CPU hogging
}

void updateDexcomData(unsigned long currentTime, unsigned long& lastUpdateTime, unsigned long long& lastDexcomReadingTime, bool isOverdueCheck) {
    // Check WiFi connection first
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected - cannot update glucose data");
        displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_WIFI_DISCONNECTED_PUBLIC);
        return;
    }
    
    // Route to LibreView when the user configured Libre credentials instead of Dexcom.
    if (isLibreUser()) {
        updateLibreData(currentTime, lastUpdateTime, lastDexcomReadingTime);
        return;
    }
    
    // Only try if we have credentials
    if (strlen(settings.dexcomUsername) > 0 && strlen(settings.dexcomPassword) > 0) {
        float current_glucose = 0;
        float previous_glucose = 0;
        char trend[20] = "";
        char timestamp[10] = "";
        unsigned long long dexcomTime = 0;
        
        // First try to fetch glucose data with existing session (if available)
        bool success = false;
        if (strlen(dexcomClient.getSessionId()) > 0) {
            // Try with existing session first
            success = dexcomClient.fetchGlucoseData(current_glucose, previous_glucose, trend, timestamp, dexcomTime, isOverdueCheck);
        }
        
        // Only re-authenticate if:
        // 1. We have no session at all, OR
        // 2. Fetch failed AND we haven't successfully fetched in the last 2 minutes
        //    (this prevents rapid re-auth during normal polling when Dexcom returns errors)
        // 3. Override: force re-auth after 3+ consecutive failures even with recent success
        // IMPORTANT: NEVER re-auth on 429 - that's rate limiting, not session issue
        bool sessionMissing = (strlen(dexcomClient.getSessionId()) == 0);
        bool recentSuccess = (g_lastSuccessfulFetchTime > 0) && 
                             ((currentTime - g_lastSuccessfulFetchTime) < 120000); // 2 minutes
        bool wasRateLimited = (dexcomClient.getLastHttpError() == 429);
        bool tooManyFailures = (g_consecutiveFetchFailures >= 3); // Force re-auth after 3 failures
        
        if (!success && wasRateLimited) {
            Serial.println("Rate limited (429) - NOT re-authenticating, session is still valid");
            g_consecutiveFetchFailures++;
        } else if (!success && (sessionMissing || !recentSuccess || tooManyFailures)) {
            if (tooManyFailures && recentSuccess) {
                Serial.printf("Forcing re-auth after %d consecutive failures despite recent success\n", g_consecutiveFetchFailures);
            } else {
                Serial.println("Session invalid or stale - re-authenticating...");
            }
            if (dexcomClient.authenticateWithRetry(false)) {
                if (dexcomClient.login(false)) {
                    success = dexcomClient.fetchGlucoseData(current_glucose, previous_glucose, trend, timestamp, dexcomTime, isOverdueCheck);
                }
            }
            if (!success) {
                g_consecutiveFetchFailures++;
            }
        } else if (!success && recentSuccess) {
            g_consecutiveFetchFailures++;
            Serial.printf("Fetch failed (failure #%d) but had recent success - skipping re-auth\n", g_consecutiveFetchFailures);
        }
        
        // If we successfully got data, update the display
        if (success) {
            bool isNewReading = (dexcomTime > lastDexcomReadingTime);
            
            // Track successful fetch time for grace period logic
            g_lastSuccessfulFetchTime = currentTime;
            g_consecutiveFetchFailures = 0; // Reset failure counter on any successful fetch
            
            // Only clear diagnostic and update display if the reading is BOTH fresh AND new.
            // If the API returns the same old reading, keep "No Data" showing.
            time_t nowCheck = time(nullptr);
            unsigned long long nowMs = ((unsigned long long)nowCheck) * 1000ULL;
            unsigned long readingAgeSec = (nowMs > dexcomTime) ? (unsigned long)((nowMs - dexcomTime) / 1000ULL) : 0;
            
            if (isNewReading && readingAgeSec < DisplayManager::STALE_AFTER_SECONDS_PUBLIC) {
                // New AND fresh reading - clear diagnostic and update display
                Serial.printf("New fresh reading (age %lus) - updating display\n", readingAgeSec);
                displayManager.clearDiagnosticState();
                displayManager.setGlucoseData(current_glucose, previous_glucose, trend, timestamp, dexcomTime);
            } else if (!isNewReading && readingAgeSec < DisplayManager::STALE_AFTER_SECONDS_PUBLIC) {
                // Same reading, still within freshness window - leave display as-is
                // Don't clear diagnostic (if "No Data" is showing, keep it)
                Serial.printf("Same reading re-fetched (age %lus) - no display change\n", readingAgeSec);
            } else {
                // Reading is stale (>= 6 min) - actively enforce "No Data"
                Serial.printf("Fetch succeeded but reading is %lu seconds old (stale) - enforcing No Data\n", readingAgeSec);
                displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_DATA_STALE_PUBLIC);
            }
            lastUpdateTime = currentTime;
            
            if (isNewReading) {
                lastDexcomReadingTime = dexcomTime;
                g_lastNewReadingReceivedTime = currentTime;  // Track when we actually received this reading
                g_lastNoNewReadingFetchTime = 0;
                {
                    time_t fetchNow = time(nullptr);
                    unsigned long long fetchEpochMillis = ((unsigned long long)fetchNow) * 1000ULL;
                    unsigned long ageAtFetchSec = (fetchEpochMillis > dexcomTime)
                                                      ? (unsigned long)((fetchEpochMillis - dexcomTime) / 1000ULL)
                                                      : 0;
                    g_lastReadingAgeAtFetchSec = (ageAtFetchSec <= FRESH_READING_MAX_AGE_SEC) ? 0 : ageAtFetchSec;
                }
                if (dexcomTime > 0) {
                    // Next boundary is one reading-interval (+5s buffer) after this reading
                    unsigned long long interval = (unsigned long long)readingIntervalMs();
                    unsigned long long boundaryBase = (dexcomTime / interval) + 1ULL;
                    g_nextReadingBoundaryEpochMs = (boundaryBase * interval) + 5000ULL;
                }
                // Refresh glucose history while the inline graph is on screen so it
                // auto-updates with each new reading. This block only runs on a genuinely
                // new reading (already throttled to the reading cadence), so every new
                // reading should refresh its dots.
                if (displayManager.isDisplayingInlineGraph()) {
                    int count = 0;
                    int graphInterval = displayManager.getInlineGraphIntervalMin();
                    if (dexcomClient.fetchGlucoseHistory(glucoseHistory, count, graphInterval)) {
                        glucoseHistoryCount = count;
                        g_lastHistoryFetchTime = currentTime;
                        displayManager.forceFullUpdate();  // redraw the graph with the new data
                        Serial.printf("Loaded %d glucose history readings\n", count);
                    }
                }
            }
        }
    }
}

// Fetch the latest glucose reading from LibreView (LibreLinkUp) and update the display.
// Mirrors the success/display logic of updateDexcomData() but uses the LibreView client.
// Re-auth is throttled with exponential backoff (and skipped on 429) so a failing provider
// can't get the account rate-limited; the token is also refreshed proactively before expiry
// via libreClient.isReady().
void updateLibreData(unsigned long currentTime, unsigned long& lastUpdateTime, unsigned long long& lastReadingTime) {
    float current_glucose = 0;
    float previous_glucose = 0;
    char trend[20] = "";
    char timestamp[10] = "";
    unsigned long long readingTime = 0;
    
    // The /graph endpoint returns the latest reading AND the history array in one payload.
    // When the inline graph is on screen, ask fetchGlucoseData() to populate the history from
    // that same response so we never download the (large) payload twice per reading.
    bool wantHistory = displayManager.isDisplayingInlineGraph();
    int graphInterval = displayManager.getInlineGraphIntervalMin();
    int historyCount = 0;
    GlucoseReading* historyPtr = wantHistory ? glucoseHistory : nullptr;
    int* historyCountPtr = wantHistory ? &historyCount : nullptr;
    
    // Try with the existing session first.
    bool success = false;
    if (libreClient.isReady()) {
        success = libreClient.fetchGlucoseData(current_glucose, previous_glucose, trend, timestamp,
                                               readingTime, historyPtr, historyCountPtr,
                                               MAX_GLUCOSE_HISTORY, graphInterval);
    }
    
    // On failure, re-authenticate - but throttle it. Hammering Abbott's login endpoint on
    // every failed poll (every ~10s during an outage) can get the account rate-limited or
    // temporarily locked, so we back off exponentially and never re-auth on a 429.
    if (!success) {
        int httpErr = libreClient.getLastHttpError();
        if (httpErr == 429) {
            // Rate limited - the session is still valid, so do NOT re-authenticate.
            g_consecutiveFetchFailures++;
            Serial.println("LibreView: rate limited (429) - backing off, not re-authenticating");
            return;
        }
        
        // Backoff between re-auth attempts: 0s, 30s, 60s, 120s, 240s, capped at 300s.
        unsigned long backoffMs = 0;
        if (g_libreConsecutiveAuthFailures > 0) {
            backoffMs = 30000UL * (1UL << (unsigned)min(g_libreConsecutiveAuthFailures - 1, 4));
            if (backoffMs > 300000UL) backoffMs = 300000UL;
        }
        if (g_libreLastAuthAttemptTime != 0 && (currentTime - g_libreLastAuthAttemptTime) < backoffMs) {
            g_consecutiveFetchFailures++;
            Serial.printf("LibreView: skipping re-auth (backoff %lums active, auth failure #%d)\n",
                          backoffMs, g_libreConsecutiveAuthFailures);
            return;
        }
        
        Serial.println("LibreView: session not ready or fetch failed - re-authenticating...");
        g_libreLastAuthAttemptTime = currentTime;
        if (libreClient.authenticate()) {
            g_libreConsecutiveAuthFailures = 0;
            success = libreClient.fetchGlucoseData(current_glucose, previous_glucose, trend, timestamp,
                                                   readingTime, historyPtr, historyCountPtr,
                                                   MAX_GLUCOSE_HISTORY, graphInterval);
        } else {
            g_libreConsecutiveAuthFailures++;
        }
    }
    
    if (success) {
        g_lastSuccessfulFetchTime = currentTime;
        g_consecutiveFetchFailures = 0;
        g_libreConsecutiveAuthFailures = 0;
        
        // History (if requested) was already populated from the same payload as the reading.
        if (wantHistory && historyCount > 0) {
            glucoseHistoryCount = historyCount;
            g_lastHistoryFetchTime = currentTime;
        }
        
        bool isNewReading = (readingTime > lastReadingTime);
        
        // Differential = change from the PREVIOUS live reading. We must NOT use graphData for
        // this: graphData holds 15-minute averages that lag the live value, which produced
        // wildly wrong deltas (e.g. a 153->146 drop showing as "+34"). Instead we remember the
        // last glucoseMeasurement value and diff against it for a true reading-to-reading delta.
        float prevForDiff = (g_librePreviousGlucose > 0.0f) ? g_librePreviousGlucose : current_glucose;
        
        // Respect reading freshness using the real FactoryTimestamp age (same as Dexcom).
        // Only show the value when it is BOTH new AND fresh; enforce "No Data" once stale.
        time_t nowCheck = time(nullptr);
        unsigned long long nowMs = ((unsigned long long)nowCheck) * 1000ULL;
        unsigned long readingAgeSec = (nowMs > readingTime) ? (unsigned long)((nowMs - readingTime) / 1000ULL) : 0;
        
        if (isNewReading && readingAgeSec < DisplayManager::STALE_AFTER_SECONDS_PUBLIC) {
            Serial.printf("LibreView: new fresh reading (age %lus) - updating display\n", readingAgeSec);
            displayManager.clearDiagnosticState();
            displayManager.setGlucoseData(current_glucose, prevForDiff, trend, timestamp, readingTime);
        } else if (!isNewReading && readingAgeSec < DisplayManager::STALE_AFTER_SECONDS_PUBLIC) {
            Serial.printf("LibreView: same reading re-fetched (age %lus) - no display change\n", readingAgeSec);
        } else {
            Serial.printf("LibreView: reading is %lus old (stale) - enforcing No Data\n", readingAgeSec);
            displayManager.setDiagnosticState(DisplayManager::DIAGNOSTIC_DATA_STALE_PUBLIC);
        }
        lastUpdateTime = currentTime;
        
        if (isNewReading) {
            lastReadingTime = readingTime;
            g_lastNewReadingReceivedTime = currentTime;
            g_lastReadingAgeAtFetchSec = 0;
            g_librePreviousGlucose = current_glucose;  // remember for the next reading's differential
            if (readingTime > 0) {
                unsigned long long interval = (unsigned long long)readingIntervalMs();
                unsigned long long boundaryBase = (readingTime / interval) + 1ULL;
                g_nextReadingBoundaryEpochMs = (boundaryBase * interval) + 5000ULL;
            }
            // The history for the inline graph was already refreshed (from the same /graph
            // payload) above, so just trigger a redraw to show the new reading.
            if (wantHistory && historyCount > 0) {
                displayManager.forceFullUpdate();
                Serial.printf("LibreView: refreshed %d history readings (from same fetch)\n", historyCount);
            }
        }
    } else {
        g_consecutiveFetchFailures++;
        Serial.printf("LibreView: update failed (failure #%d)\n", g_consecutiveFetchFailures);
    }
}

// Debug function to handle serial commands for testing
void checkSerialDebugCommands() {
    static unsigned long lastSerialCheck = 0;
    if (millis() - lastSerialCheck < 500) {
        // Only check every 500ms to avoid flooding
        return;
    }
    lastSerialCheck = millis();
    
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim(); // trim() modifies the string in place
        
        // Handle glucose test command: "glucose:X" where X is a value or "LOW"/"HIGH"
        if (command.startsWith("glucose:")) {
            String valueStr = command.substring(8);
            valueStr.trim(); // trim() modifies the string in place
            
            Serial.print("Debug mode: Setting test glucose value to ");
            Serial.println(valueStr);
            
            float testValue;
            // Use the accessor to get the current glucose value
            float prevValue = displayManager.getCurrentGlucose();
            const char* trendArrow = "→"; // Default to flat trend
            char timestampStr[30];
            struct tm timeinfo;
            
            // Get current time for timestamp
            if (getLocalTime(&timeinfo)) {
                sprintf(timestampStr, "%02d/%02d/%04d %02d:%02d", 
                        timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_year + 1900,
                        timeinfo.tm_hour, timeinfo.tm_min);
            } else {
                strcpy(timestampStr, "01/01/2023 12:00"); // Fallback
            }
            
            // Parse glucose value
            if (valueStr.equalsIgnoreCase("LOW")) {
                testValue = 39; // Just below LOW threshold
                Serial.println("Setting to LOW test value");
            } else if (valueStr.equalsIgnoreCase("HIGH")) {
                testValue = 402; // Just above HIGH threshold
                Serial.println("Setting to HIGH test value");
            } else {
                testValue = valueStr.toFloat();
                Serial.print("Setting to numeric test value: ");
                Serial.println(testValue);
            }
            
            // Inject the test value into display manager
            displayManager.setGlucoseData(testValue, prevValue, trendArrow, timestampStr, time(NULL));
            Serial.println("Test glucose value injected. Check display...");
            
            // Help text for trend arrows
            Serial.println("\nTip: You can also set trend using: glucose:100:↑");
            Serial.println("Available trends: → (flat), ↑ (up), ↓ (down), ↑↑ (double up), ↓↓ (double down)");
        }
        // Handle glucose+trend command: "glucose:X:TREND"
        else if (command.startsWith("glucose:") && command.indexOf(':') != command.lastIndexOf(':')) {
            int firstColon = command.indexOf(':');
            int secondColon = command.indexOf(':', firstColon + 1);
            
            // Fix String handling: extract substring first, then trim
            String valueStr = command.substring(firstColon + 1, secondColon);
            valueStr.trim(); // trim() modifies the string in place
            
            String trendStr = command.substring(secondColon + 1);
            trendStr.trim(); // trim() modifies the string in place
            
            Serial.print("Debug mode: Setting glucose to ");
            Serial.print(valueStr);
            Serial.print(" with trend ");
            Serial.println(trendStr);
            
            float testValue;
            // Use accessor method to get current glucose value
            float prevValue = displayManager.getCurrentGlucose();
            char timestampStr[30];
            struct tm timeinfo;
            
            // Get current time for timestamp
            if (getLocalTime(&timeinfo)) {
                sprintf(timestampStr, "%02d/%02d/%04d %02d:%02d", 
                        timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_year + 1900,
                        timeinfo.tm_hour, timeinfo.tm_min);
            } else {
                strcpy(timestampStr, "01/01/2023 12:00"); // Fallback
            }
            
            // Parse glucose value
            if (valueStr.equalsIgnoreCase("LOW")) {
                testValue = 39; // Just below LOW threshold
            } else if (valueStr.equalsIgnoreCase("HIGH")) {
                testValue = 402; // Just above HIGH threshold
            } else {
                testValue = valueStr.toFloat();
            }
            
            // Inject the test values
            displayManager.setGlucoseData(testValue, prevValue, trendStr.c_str(), timestampStr, time(NULL));
            Serial.println("Test glucose value with custom trend injected. Check display...");
        }
        else if (command == "outage:on") {
            // Force the outage warning ON without waiting for stale data.
            // Sets the debug override so the main loop won't auto-clear on fresh readings.
            g_serverOutageDetected = true;
            g_debugForceOutage = true;
            displayManager.forceFullUpdate();
            Serial.printf("DEBUG: forced outage warning ON (provider=%s, override latched - use outage:off to clear)\n",
                          isLibreUser() ? "Libre" : "Dexcom");
        }
        else if (command == "outage:off") {
            g_serverOutageDetected = false;
            g_debugForceOutage = false;
            g_lastOutageCheckTime = 0;
            displayManager.forceFullUpdate();
            Serial.println("DEBUG: cleared outage warning and debug override");
        }
        else if (command == "outage:check") {
            // Trigger an immediate status page poll regardless of staleness
            Serial.println("DEBUG: forcing immediate status page check...");
            checkServerStatusForOutage();
            Serial.printf("DEBUG: g_serverOutageDetected = %s\n",
                          g_serverOutageDetected ? "true" : "false");
        }
        else if (command == "help") {
            Serial.println("\nAvailable debug commands:\n");
            Serial.println("glucose:X      - Set glucose to X (number, LOW, or HIGH)");
            Serial.println("glucose:X:T    - Set glucose to X with trend T");
            Serial.println("                Trends: -> (flat), ^ (up), v (down), ^^ (double up), vv (double down)");
            Serial.println("outage:on      - Force the server outage warning ON (testing)");
            Serial.println("outage:off     - Clear the server outage warning");
            Serial.println("outage:check   - Run an immediate status page poll (Dexcom or Libre)");
            Serial.println("help           - Show this help menu");
        }
    }
}

// Determine whether the user is configured for LibreView (vs. Dexcom).
// Used by Down Detector to pick which provider's status page to query.
// Note: The setup flow guarantees only one provider is configured at a time.
static bool isLibreUser() {
    return strlen(settings.libreViewEmail) > 0 && strlen(settings.libreViewPassword) > 0;
}

// Expected interval between new readings for the active provider, in milliseconds.
// Dexcom Share publishes a new value every 5 minutes; LibreView (LibreLinkUp) every ~1 minute.
// Used to align the proactive polling cadence and reading boundary with the real data source.
static unsigned long readingIntervalMs() {
    return isLibreUser() ? 60000UL : 300000UL;
}

// Seconds after a reading at which to begin proactive polling for the next one
// (one reading interval + a 5 second buffer to let the provider process the new value).
static unsigned long proactiveStartSeconds() {
    return isLibreUser() ? 65UL : 305UL;
}

// Poll the appropriate provider's status page and set g_serverOutageDetected if any
// monitored service is reported non-operational. Called after data has been stale > 1 hour.
// On fetch failure (network/HTTP error), the existing flag state is preserved (no false positives).
static void checkServerStatusForOutage() {
    g_lastOutageCheckTime = millis();
    
    ServiceStatus entries[4];
    int count = 0;
    bool ok = isLibreUser()
        ? fetchLibreStatus(entries, 4, count)
        : fetchDexcomStatus(entries, 4, count);
    
    if (!ok || count == 0) {
        Serial.println("Outage check: fetch failed - preserving previous state");
        return;
    }
    
    // Trip the flag if ANY service is non-operational (including degraded/partial/major/maintenance).
    bool outage = false;
    for (int i = 0; i < count; i++) {
        const char* s = entries[i].status;
        if (strlen(s) > 0 && strcmp(s, "operational") != 0) {
            outage = true;
            Serial.printf("Outage check: %s reports '%s'\n", entries[i].label, s);
        }
    }
    
    if (outage != g_serverOutageDetected) {
        g_serverOutageDetected = outage;
        Serial.printf("Outage check: state changed -> %s\n", outage ? "OUTAGE" : "all operational");
        // Force redraw so the bottom bar text updates immediately
        displayManager.forceFullUpdate();
    } else {
        Serial.printf("Outage check: %s (no change)\n", outage ? "still outage" : "all operational");
    }
}

// Internal helper: GETs a StatusPage-format components.json and matches given names.
// `wantedNames`: array of exact component names to look up (group-level aggregates).
// `outLabels`: array of display labels (1:1 with wantedNames), assigned to entries on match.
// `out`: pre-allocated ServiceStatus array (size >= wantedCount).
// Returns true if all wanted components were found.
static bool fetchStatusPageComponents(const char* url,
                                      const char* const* wantedNames,
                                      const char* const* outLabels,
                                      int wantedCount,
                                      ServiceStatus* out,
                                      int& outCount) {
    outCount = 0;
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Down Detector: WiFi not connected");
        return false;
    }
    
    WiFiClientSecure client;
    client.setInsecure(); // Skip cert verification
    
    HTTPClient https;
    https.setTimeout(10000);
    https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    
    Serial.printf("Down Detector: GET %s\n", url);
    
    if (!https.begin(client, url)) {
        Serial.println("Down Detector: https.begin failed");
        return false;
    }
    
    int httpCode = https.GET();
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("Down Detector: HTTP %d\n", httpCode);
        https.end();
        return false;
    }
    
    String payload = https.getString();
    https.end();
    
    // Filter the response down to just components[].name and components[].status.
    StaticJsonDocument<128> filter;
    JsonObject compFilter = filter["components"].createNestedObject();
    compFilter["name"] = true;
    compFilter["status"] = true;
    
    // Libre payload is larger (~12KB raw, ~6KB filtered with ~50 components).
    DynamicJsonDocument doc(8192);
    DeserializationError err = deserializeJson(doc, payload, DeserializationOption::Filter(filter));
    if (err) {
        Serial.printf("Down Detector: JSON parse error: %s\n", err.c_str());
        return false;
    }
    
    JsonArray components = doc["components"].as<JsonArray>();
    if (components.isNull()) {
        Serial.println("Down Detector: no components array");
        return false;
    }
    
    // Track which wanted entries were matched (parallel to wantedNames).
    bool found[8] = {false}; // supports up to 8 entries
    if (wantedCount > 8) wantedCount = 8;
    
    for (JsonObject c : components) {
        const char* name = c["name"] | "";
        const char* status = c["status"] | "";
        for (int i = 0; i < wantedCount; i++) {
            if (!found[i] && strcmp(name, wantedNames[i]) == 0) {
                out[i].label = outLabels[i];
                strncpy(out[i].status, status, sizeof(out[i].status) - 1);
                out[i].status[sizeof(out[i].status) - 1] = '\0';
                found[i] = true;
                break;
            }
        }
    }
    
    // Always emit entries in the wanted order, even if some are missing.
    outCount = wantedCount;
    bool allFound = true;
    for (int i = 0; i < wantedCount; i++) {
        if (!found[i]) {
            out[i].label = outLabels[i];
            out[i].status[0] = '\0';
            allFound = false;
            Serial.printf("Down Detector: missing component '%s'\n", wantedNames[i]);
        } else {
            Serial.printf("Down Detector: %s = %s\n", out[i].label, out[i].status);
        }
    }
    
    return allFound;
}

// Fetch Dexcom server status for Follow App, G7 App, and Clarity from status.dexcom.com.
bool fetchDexcomStatus(ServiceStatus* out, int maxCount, int& count) {
    static const char* const wanted[] = { "Follow App", "G7 App", "Clarity" };
    static const char* const labels[] = { "Follow:",    "G7:",    "Clarity:" };
    const int wantedCount = 3;
    if (maxCount < wantedCount) { count = 0; return false; }
    return fetchStatusPageComponents("https://status.dexcom.com/api/v2/components.json",
                                     wanted, labels, wantedCount, out, count);
}

// Fetch Libre/Abbott server status for the 4 top-level service categories.
bool fetchLibreStatus(ServiceStatus* out, int maxCount, int& count) {
    // These names are the EXACT group-level component names from status.freestyle.abbott.
    // The Libre mobile app group has an unusually long name; we match it exactly.
    static const char* const wanted[] = {
        "Libre App, FreeStyle LibreLink, FreeStyle Libre 2, FreeStyle Libre 3 Mobile App*",
        "LibreView Web App",
        "LibreLinkUp Mobile App",
        "Libre Data Sharing API"
    };
    static const char* const labels[] = {
        "Libre App:",
        "LibreView:",
        "LinkUp:",
        "Data API:"
    };
    const int wantedCount = 4;
    if (maxCount < wantedCount) { count = 0; return false; }
    return fetchStatusPageComponents("https://status.freestyle.abbott/api/v2/components.json",
                                     wanted, labels, wantedCount, out, count);
}

// Automatic timezone detection using ip-api.com.
// WorldTimeAPI was previously used but has suffered frequent, extended outages
// (confirmed repeatedly down through 2025), which is why time/timezone setup kept
// failing. ip-api.com is a long-established, reliable IP-geolocation service whose
// free tier returns the current UTC offset (already including DST) directly in seconds.
bool fetchAutoTimezone() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected - cannot fetch timezone");
        return false;
    }
    
    Serial.println("Fetching automatic timezone from ip-api.com...");
    Serial.printf("WiFi Signal Strength: %d\n", WiFi.RSSI());
    
    // Try multiple times for better reliability
    const int MAX_RETRIES = 3;
    const int RETRY_DELAY_MS = 2000; // 2 seconds between retries
    // Free tier is HTTP-only; timezone data is not sensitive, so plain HTTP is fine.
    // 'fields' trims the response to just what we parse (status, message, timezone, offset).
    const char* url = "http://ip-api.com/json/?fields=status,message,timezone,offset";
    
    // Declare variables at function level so they're accessible outside the loop
    bool success = false;
    String payload;
    
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        Serial.printf("Timezone fetch attempt %d of %d\n", attempt, MAX_RETRIES);
        
        if (attempt > 1) {
            Serial.printf("Waiting %d seconds before retry...\n", RETRY_DELAY_MS / 1000);
            delay(RETRY_DELAY_MS);
            
            // Check WiFi connection before retry
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("WiFi disconnected during retry - aborting");
                return false;
            }
        }
        
        success = false;
        
        WiFiClient client;
        HTTPClient http;
        http.setTimeout(10000);
        
        if (http.begin(client, url)) {
            http.addHeader("User-Agent", "ESP32-SugrBee");
            
            Serial.println("Sending request...");
            int httpCode = http.GET();
            Serial.printf("HTTP Response Code: %d\n", httpCode);
            
            if (httpCode == HTTP_CODE_OK) {
                payload = http.getString();
                success = true;
            } else {
                Serial.printf("Request failed with code %d\n", httpCode);
            }
            
            http.end();
        } else {
            Serial.println("Failed to begin HTTP connection");
        }
        
        // If we got a successful response, process it and exit retry loop
        if (success) {
            Serial.printf("Timezone fetch successful on attempt %d!\n", attempt);
            break; // Exit the retry loop on success
        } else {
            Serial.printf("Attempt %d failed\n", attempt);
            if (attempt == MAX_RETRIES) {
                Serial.println("All timezone fetch attempts failed");
            }
        }
    } // End of retry loop
    
    if (!success) {
        Serial.println("ip-api.com request failed - all connection attempts failed");
        return false;
    }
    
    Serial.println("ip-api.com response:");
    Serial.println(payload);
    
    // Parse JSON response
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
        Serial.print("Failed to parse timezone JSON: ");
        Serial.println(error.c_str());
        return false;
    }
    
    // ip-api returns {"status":"success"|"fail", ...}. Bail out on a failure status
    // (e.g., rate-limited or private IP) so we keep the previously saved offset.
    const char* status = doc["status"] | "";
    if (strcmp(status, "success") != 0) {
        const char* msg = doc["message"] | "unknown error";
        Serial.printf("ip-api.com returned non-success status: %s\n", msg);
        return false;
    }
    
    const char* timezone = doc["timezone"] | "Unknown";
    // 'offset' is the current UTC offset in SECONDS and already includes DST.
    long offset_seconds = doc["offset"] | 0L;
    int total_offset_hours = (int)(offset_seconds / 3600);
    
    Serial.printf("Detected timezone: %s\n", timezone);
    Serial.printf("UTC offset: %ld seconds (%d hours)\n", offset_seconds, total_offset_hours);
    
    // Persist the detected offset.
    settings.timeZoneOffset = total_offset_hours;
    settings.save();
    
    // Re-apply the clock with the corrected offset. Because ip-api's offset already
    // includes DST, a fixed-offset configTime() is correct; the daily timezone re-check
    // (every 24h) picks up future DST transitions and travel.
    int gmtOffset_sec = total_offset_hours * 3600;
    configTime(gmtOffset_sec, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    
    displayManager.updateTimeDisplay();
    Serial.println("Timezone configured successfully!");
    return true;
}

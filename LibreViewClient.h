#ifndef LIBREVIEW_CLIENT_H
#define LIBREVIEW_CLIENT_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <mbedtls/sha256.h>
#include "Settings.h"

// LibreView API base URL (US default)
#define LIBRE_API_BASE "https://api.libreview.io"
// Regional URLs use format: https://api-{region}.libreview.io

// API Endpoints
#define LIBRE_LOGIN_ENDPOINT "/llu/auth/login"
#define LIBRE_CONNECTIONS_ENDPOINT "/llu/connections"
#define LIBRE_GRAPH_ENDPOINT "/llu/connections/%s/graph"

// Required headers.
// NOTE: Abbott enforces a MINIMUM `version` value. As of the Oct 2025 update the API
// rejects anything below 4.16.0 with HTTP 403 ({"data":{"minimumVersion":"4.16.0"},
// "status":920}) on authenticated endpoints, so keep this current.
#define LIBRE_PRODUCT "llu.android"
#define LIBRE_VERSION "4.16.0"

// Network timeout applied to every LibreView HTTP request so a hung connection
// can never block the main loop (and freeze the display/touch) indefinitely.
#define LIBRE_HTTP_TIMEOUT_MS 15000

// LibreLinkUp TrendArrow is a 1-5 scale matching FreeStyle Libre's five on-screen arrows:
//   1 = Falling quickly (down), 2 = Falling (45 down), 3 = Stable (flat),
//   4 = Rising (45 up),         5 = Rising quickly (up)
// Libre uses single + diagonal arrows only (never double arrows), so we map to the
// matching Dexcom-style direction strings the display understands (see
// DEXCOM_TREND_DIRECTIONS / drawTrendIndicator() in the main sketch). Mapping value 2/4 to
// the 45-degree arrows keeps our display icon-faithful to the LibreLinkUp app.
static const char* const LIBRE_TREND_ARROWS[] = {
    "None",          // 0 - Unknown/None
    "SingleDown",    // 1 - Falling quickly   (straight down)
    "FortyFiveDown", // 2 - Falling           (45 down)
    "Flat",          // 3 - Stable            (flat)
    "FortyFiveUp",   // 4 - Rising            (45 up)
    "SingleUp"       // 5 - Rising quickly    (straight up)
};

extern Settings settings;
extern TFT_eSPI tft;

class LibreViewClient {
private:
    char authToken[1024] = "";
    char patientId[64] = "";
    char accountIdHash[65] = "";      // SHA-256 hex of the login user id (required as the Account-Id header)
    unsigned long tokenExpires = 0;   // Unix epoch seconds when the JWT expires (0 = unknown)
    bool isAuthenticated = false;
    char baseUrl[64];
    int lastHttpError = 0;            // HTTP status of the most recent request (for backoff decisions)
    char libreVersion[16] = LIBRE_VERSION;  // 'version' header sent to Abbott. Defaults to the
                                            // compiled-in value but is auto-updated (and persisted)
                                            // if the API reports a higher required minimumVersion.
    
    // Compute the lowercase hex SHA-256 of `input` into `outHex` (must be >= 65 bytes).
    // LibreLinkUp requires the SHA-256 of the account id to be sent as the Account-Id
    // header on authenticated requests; without it the API returns HTTP 403.
    void computeSha256Hex(const char* input, char* outHex) {
        unsigned char hash[32];
        mbedtls_sha256((const unsigned char*)input, strlen(input), hash, 0); // 0 = SHA-256 (not 224)
        static const char* hexd = "0123456789abcdef";
        for (int i = 0; i < 32; i++) {
            outHex[i * 2]     = hexd[(hash[i] >> 4) & 0x0F];
            outHex[i * 2 + 1] = hexd[hash[i] & 0x0F];
        }
        outHex[64] = '\0';
    }
    
    // Set base URL based on region string
    void setBaseUrl() {
        if (strlen(settings.libreViewRegion) > 0) {
            snprintf(baseUrl, sizeof(baseUrl), "https://api-%s.libreview.io", settings.libreViewRegion);
            Serial.printf("LibreView: Using %s server: %s\n", settings.libreViewRegion, baseUrl);
        } else {
            strncpy(baseUrl, LIBRE_API_BASE, sizeof(baseUrl) - 1);
            baseUrl[sizeof(baseUrl) - 1] = '\0';
            Serial.println("LibreView: Using US server");
        }
    }
    
    // Add required headers to HTTP client
    void addHeaders(HTTPClient& http, bool includeAuth = false) {
        http.addHeader("Content-Type", "application/json");
        http.addHeader("Accept", "application/json");
        http.addHeader("product", LIBRE_PRODUCT);
        http.addHeader("version", libreVersion);
        // Note: Do NOT send Accept-Encoding: gzip - ESP32 HTTPClient cannot decompress it
        http.addHeader("Cache-Control", "no-cache");
        http.addHeader("Connection", "Keep-Alive");
        
        if (includeAuth && strlen(authToken) > 0) {
            String authHeader = "Bearer ";
            authHeader += authToken;
            http.addHeader("Authorization", authHeader.c_str());
            // Required since Abbott's API update: SHA-256 of the account id. Missing this
            // header is what causes /connections and /graph to fail with HTTP 403.
            if (strlen(accountIdHash) > 0) {
                http.addHeader("Account-Id", accountIdHash);
            }
        }
    }
    
    // Convert LibreView trend arrow (1-5) to a Dexcom-style direction string
    const char* getTrendArrow(int trendValue) {
        if (trendValue >= 0 && trendValue <= 5) {
            return LIBRE_TREND_ARROWS[trendValue];
        }
        return "Flat"; // Default to stable
    }
    
    // Convert a UTC calendar date/time to Unix epoch seconds.
    // Portable (Howard Hinnant's days-from-civil algorithm) so it does not depend on
    // timegm() being available, and unaffected by the device's configured timezone.
    long long utcToEpoch(int year, int mon, int mday, int hour, int minute, int sec) {
        int y = year;
        y -= (mon <= 2);
        long long era = (y >= 0 ? y : y - 399) / 400;
        unsigned yoe = (unsigned)(y - era * 400);
        unsigned doy = (153u * (unsigned)(mon + (mon > 2 ? -3 : 9)) + 2u) / 5u + (unsigned)mday - 1u;
        unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
        long long days = era * 146097LL + (long long)doe - 719468LL;
        return days * 86400LL + (long long)hour * 3600LL + (long long)minute * 60LL + (long long)sec;
    }
    
    // Parse a LibreView timestamp string ("M/D/YYYY h:mm:ss AM/PM") into UTC epoch milliseconds.
    // LibreView's FactoryTimestamp is in UTC. Returns 0 on parse failure.
    unsigned long long parseLibreTimestampUtcMs(const char* ts) {
        if (!ts || strlen(ts) == 0) return 0;
        int mo = 0, d = 0, y = 0, h = 0, mi = 0, s = 0;
        char ampm[4] = {0};
        int n = sscanf(ts, "%d/%d/%d %d:%d:%d %3s", &mo, &d, &y, &h, &mi, &s, ampm);
        if (n < 6) return 0; // Need at least date + h:m:s
        // Convert 12-hour clock to 24-hour when an AM/PM marker is present
        if (ampm[0] == 'P' || ampm[0] == 'p') { if (h != 12) h += 12; }
        else if (ampm[0] == 'A' || ampm[0] == 'a') { if (h == 12) h = 0; }
        if (mo < 1 || mo > 12 || d < 1 || d > 31) return 0;
        long long epoch = utcToEpoch(y, mo, d, h, mi, s);
        if (epoch <= 0) return 0;
        return (unsigned long long)epoch * 1000ULL;
    }
    
    // Abbott rejects requests whose 'version' header is below a server-enforced minimum, returning
    // a body like {"data":{"minimumVersion":"4.17.0"},"status":920}. This detects that, adopts the
    // required version at runtime, and PERSISTS it so the device self-heals across reboots without a
    // reflash. Returns true only when a new, valid version was adopted (so the caller can retry once).
    bool adoptRequiredVersion(const String& body) {
        JsonDocument doc;
        if (deserializeJson(doc, body)) return false; // empty/non-JSON body (e.g. HTML) - nothing to do
        
        const char* minV = doc["data"]["minimumVersion"] | "";
        if (strlen(minV) == 0) return false; // not a version rejection
        
        // Loud diagnostic regardless of what we do next, so it's unmistakable in the logs.
        Serial.printf("LibreView: *** API requires a newer app version: minimumVersion=%s "
                      "(we are sending %s) ***\n", minV, libreVersion);
        
        // Sanity-check the value before trusting it: must look like a dotted version (e.g. 4.17.0)
        // and fit our buffer, so a malformed/hostile body can't poison the persisted setting.
        int a = 0, b = 0, c = 0;
        if (sscanf(minV, "%d.%d.%d", &a, &b, &c) != 3 || strlen(minV) >= sizeof(libreVersion)) {
            Serial.printf("LibreView: ignoring malformed minimumVersion '%s' - "
                          "update LIBRE_VERSION manually if logins keep failing\n", minV);
            return false;
        }
        if (strcmp(minV, libreVersion) == 0) return false; // already sending it; not a version issue
        
        Serial.printf("LibreView: auto-adopting version %s (was %s) and saving it\n", minV, libreVersion);
        strncpy(libreVersion, minV, sizeof(libreVersion) - 1);
        libreVersion[sizeof(libreVersion) - 1] = '\0';
        strncpy(settings.libreViewVersion, libreVersion, sizeof(settings.libreViewVersion) - 1);
        settings.libreViewVersion[sizeof(settings.libreViewVersion) - 1] = '\0';
        settings.save();
        return true;
    }
    
    // Fill `history` (newest-first) from Libre's graphData, sampled at ~intervalMinutes spacing
    // by the points' own Timestamps. graphData is chronological (oldest..newest) and covers the
    // last ~12h, so a 15-min interval yields a ~6h view from the SAME payload we already fetched.
    // If liveValue >= 0 it becomes the newest dot (so the graph's right edge matches the live
    // reading shown as the big number). Returns the number of points written.
    int fillSampledHistory(JsonArray graphData, GlucoseReading* history,
                           int maxHistory, int intervalMinutes, float liveValue) {
        int n = (int)graphData.size();
        if (n == 0 || maxHistory <= 0) return 0;
        
        int count = 0;
        unsigned long long lastKeptMs = 0;
        unsigned long long intervalMs = (unsigned long long)intervalMinutes * 60000ULL;
        
        if (liveValue >= 0.0f) {
            // Anchor spacing to the newest graph point (≈ now) so we skip near-duplicate points.
            lastKeptMs = parseLibreTimestampUtcMs(graphData[n - 1]["Timestamp"] | "");
            history[count].value = liveValue;
            // Stamp the live point with the newest graph point's clock - NOT time(nullptr). The
            // graph points' timestamps come from Libre's local-time "Timestamp" field, so mixing in
            // the device's own clock here made the span label (newest - oldest) include the timezone
            // offset and inflated e.g. ~6h to ~11h. Keeping one consistent clock fixes that.
            history[count].timestamp = (unsigned long)(lastKeptMs / 1000ULL);
            count++;
        }
        
        for (int i = n - 1; i >= 0 && count < maxHistory; i--) {
            unsigned long long ts = parseLibreTimestampUtcMs(graphData[i]["Timestamp"] | "");
            // For the wide (interval > 1) view, keep a point only once it's at least one interval
            // older than the last kept point. Missing/unparseable timestamps fall through (kept).
            if (intervalMinutes > 1 && lastKeptMs != 0 && ts != 0 && (lastKeptMs - ts) < intervalMs) {
                continue;
            }
            history[count].value = graphData[i]["ValueInMgPerDl"] | 0;
            // Store the real reading time (epoch seconds) so the graph can label its true span.
            history[count].timestamp = (unsigned long)(ts / 1000ULL);
            count++;
            if (ts != 0) lastKeptMs = ts;
        }
        return count;
    }

public:
    void init() {
        setBaseUrl();
        authToken[0] = '\0';
        patientId[0] = '\0';
        accountIdHash[0] = '\0';
        isAuthenticated = false;
        // Use a previously learned 'version' (survives reboots) if we have one; otherwise the
        // compiled-in default. This is what lets the client keep working after Abbott raises the
        // minimum without needing a reflash.
        if (strlen(settings.libreViewVersion) > 0) {
            strncpy(libreVersion, settings.libreViewVersion, sizeof(libreVersion) - 1);
            libreVersion[sizeof(libreVersion) - 1] = '\0';
        } else {
            strncpy(libreVersion, LIBRE_VERSION, sizeof(libreVersion) - 1);
            libreVersion[sizeof(libreVersion) - 1] = '\0';
        }
        Serial.printf("LibreView client initialized (version header %s)\n", libreVersion);
    }
    
    // Login and get JWT token. `redirectDepth` bounds the region-redirect recursion so a
    // server that ping-pongs regions can never overflow the stack.
    bool login(int redirectDepth = 0) {
        if (redirectDepth > 3) {
            Serial.println("LibreView: Too many region redirects - aborting login");
            return false;
        }
        if (strlen(settings.libreViewEmail) == 0 || strlen(settings.libreViewPassword) == 0) {
            Serial.println("LibreView: No credentials configured");
            return false;
        }
        
        setBaseUrl();
        
        WiFiClientSecure client;
        client.setInsecure(); // Skip certificate validation
        
        HTTPClient http;
        String url = String(baseUrl) + LIBRE_LOGIN_ENDPOINT;
        
        Serial.print("LibreView: Logging in to ");
        Serial.println(url);
        
        if (!http.begin(client, url)) {
            Serial.println("LibreView: Failed to begin HTTP connection");
            return false;
        }
        
        http.setTimeout(LIBRE_HTTP_TIMEOUT_MS);
        addHeaders(http);
        
        // Create login payload
        JsonDocument doc;
        doc["email"] = settings.libreViewEmail;
        doc["password"] = settings.libreViewPassword;
        
        String payload;
        serializeJson(doc, payload);
        
        int httpCode = http.POST(payload);
        lastHttpError = httpCode;
        
        if (httpCode != 200) {
            Serial.printf("LibreView: Login failed with code %d\n", httpCode);
            String response = http.getString();
            Serial.println(response);
            http.end();
            // If the failure is a version rejection, adopt the required version and retry once.
            if (redirectDepth == 0 && adoptRequiredVersion(response)) {
                return login(redirectDepth + 1);
            }
            return false;
        }
        
        String response = http.getString();
        http.end();
        
        // Parse response
        JsonDocument responseDoc;
        DeserializationError error = deserializeJson(responseDoc, response);
        
        if (error) {
            Serial.print("LibreView: JSON parse error: ");
            Serial.println(error.c_str());
            return false;
        }
        
        // Check status
        int status = responseDoc["status"] | -1;
        if (status != 0) {
            Serial.printf("LibreView: Login returned status %d\n", status);
            return false;
        }
        
        // Check for redirect (region mismatch)
        if (responseDoc["data"]["redirect"].is<bool>() && responseDoc["data"]["redirect"].as<bool>()) {
            const char* region = responseDoc["data"]["region"] | "";
            Serial.printf("LibreView: Redirect required to region: %s\n", region);
            
            // Auto-switch to the correct region
            if (strlen(region) > 0 && strcmp(region, settings.libreViewRegion) != 0) {
                Serial.printf("LibreView: Switching to %s server\n", region);
                strncpy(settings.libreViewRegion, region, sizeof(settings.libreViewRegion) - 1);
                settings.libreViewRegion[sizeof(settings.libreViewRegion) - 1] = '\0';
                settings.save();
                setBaseUrl();
                return login(redirectDepth + 1); // Retry with correct regional server
            }
            Serial.println("LibreView: Redirect requested but no valid region provided");
            return false;
        }
        
        // Some accounts must (re)accept Terms of Use / Privacy Policy, or verify their email,
        // before the API will issue a usable session. The login response signals this with a
        // "step" object instead of an authTicket. We can't complete that flow on-device, so
        // surface a clear message telling the user to open the LibreLinkUp app.
        const char* stepType = responseDoc["data"]["step"]["type"] | "";
        if (strlen(stepType) > 0) {
            Serial.printf("LibreView: account action required (step='%s') before data access. "
                          "Open the LibreLinkUp app and accept the latest Terms/Privacy (or verify email).\n",
                          stepType);
            return false;
        }
        
        // Extract auth token
        const char* token = responseDoc["data"]["authTicket"]["token"] | "";
        if (strlen(token) == 0) {
            Serial.println("LibreView: No token in response");
            return false;
        }
        
        strncpy(authToken, token, sizeof(authToken) - 1);
        authToken[sizeof(authToken) - 1] = '\0';
        
        tokenExpires = responseDoc["data"]["authTicket"]["expires"] | 0;
        
        // Compute the Account-Id header value (SHA-256 of the login user id). Abbott's API
        // requires this on authenticated requests; without it /connections returns HTTP 403.
        const char* userId = responseDoc["data"]["user"]["id"] | "";
        if (strlen(userId) > 0) {
            computeSha256Hex(userId, accountIdHash);
            Serial.println("LibreView: Account-Id hash computed from user id");
        } else {
            accountIdHash[0] = '\0';
            Serial.println("LibreView: WARNING - no data.user.id in login response; Account-Id header will be missing");
        }
        
        isAuthenticated = true;
        
        Serial.println("LibreView: Login successful");
        return true;
    }
    
    // Get patient connections and store patientId. `attempt` bounds the one-shot retry we do
    // after auto-adopting a newer required version (so we can never loop).
    bool getConnections(int attempt = 0) {
        if (!isAuthenticated || strlen(authToken) == 0) {
            Serial.println("LibreView: Not authenticated");
            return false;
        }
        
        WiFiClientSecure client;
        client.setInsecure();
        
        HTTPClient http;
        String url = String(baseUrl) + LIBRE_CONNECTIONS_ENDPOINT;
        
        Serial.print("LibreView: Getting connections from ");
        Serial.println(url);
        
        if (!http.begin(client, url)) {
            Serial.println("LibreView: Failed to begin HTTP connection");
            return false;
        }
        
        http.setTimeout(LIBRE_HTTP_TIMEOUT_MS);
        addHeaders(http, true);
        
        int httpCode = http.GET();
        lastHttpError = httpCode;
        
        if (httpCode != 200) {
            String body = http.getString();
            Serial.printf("LibreView: Get connections failed with code %d\n", httpCode);
            Serial.println(body); // log body (often reveals e.g. minimumVersion)
            http.end();
            // Self-heal: if Abbott rejected our version, adopt the required one and retry once.
            if (attempt == 0 && adoptRequiredVersion(body)) {
                return getConnections(attempt + 1);
            }
            return false;
        }
        
        String response = http.getString();
        http.end();
        
        // Parse response
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, response);
        
        if (error) {
            Serial.print("LibreView: JSON parse error: ");
            Serial.println(error.c_str());
            return false;
        }
        
        // Check status
        int status = doc["status"] | -1;
        if (status != 0) {
            Serial.printf("LibreView: Connections returned status %d\n", status);
            return false;
        }
        
        // Get first patient ID
        JsonArray data = doc["data"].as<JsonArray>();
        if (data.size() == 0) {
            Serial.println("LibreView: No connections found");
            return false;
        }
        
        const char* pid = data[0]["patientId"] | "";
        if (strlen(pid) == 0) {
            Serial.println("LibreView: No patientId in first connection");
            return false;
        }
        
        strncpy(patientId, pid, sizeof(patientId) - 1);
        patientId[sizeof(patientId) - 1] = '\0';
        
        Serial.printf("LibreView: Found patient ID: %s\n", patientId);
        return true;
    }
    
    // Fetch glucose data. The LibreView /graph endpoint returns BOTH the latest
    // measurement and the full history array in a single payload, so callers that also
    // want the graph history can pass a non-null `history`/`historyCount` to have it
    // populated from this same response - avoiding a second download of the same data.
    bool fetchGlucoseData(float& current_glucose, float& previous_glucose, 
                          char* trend, char* timestamp, unsigned long long& readingTime,
                          GlucoseReading* history = nullptr, int* historyCount = nullptr,
                          int maxHistory = 24, int intervalMinutes = 1, int attempt = 0) {
        if (!isAuthenticated || strlen(patientId) == 0) {
            Serial.println("LibreView: Not ready to fetch glucose data");
            return false;
        }
        
        WiFiClientSecure client;
        client.setInsecure();
        
        HTTPClient http;
        char endpoint[128];
        snprintf(endpoint, sizeof(endpoint), LIBRE_GRAPH_ENDPOINT, patientId);
        String url = String(baseUrl) + endpoint;
        
        Serial.print("LibreView: Fetching glucose from ");
        Serial.println(url);
        
        if (!http.begin(client, url)) {
            Serial.println("LibreView: Failed to begin HTTP connection");
            return false;
        }
        
        http.setTimeout(LIBRE_HTTP_TIMEOUT_MS);
        addHeaders(http, true);
        
        int httpCode = http.GET();
        lastHttpError = httpCode;
        
        if (httpCode != 200) {
            String body = http.getString();
            Serial.printf("LibreView: Get glucose failed with code %d\n", httpCode);
            Serial.println(body); // log body (often reveals e.g. minimumVersion)
            http.end();
            // Self-heal: if Abbott rejected our version, adopt the required one and retry once.
            if (attempt == 0 && adoptRequiredVersion(body)) {
                return fetchGlucoseData(current_glucose, previous_glucose, trend, timestamp,
                                        readingTime, history, historyCount, maxHistory,
                                        intervalMinutes, attempt + 1);
            }
            return false;
        }
        
        // getString() de-chunks the (Transfer-Encoding: chunked) response for us; reading the
        // raw stream directly would feed chunk-framing bytes to the parser (InvalidInput).
        String response = http.getString();
        http.end();
        
        // The /graph payload is large (full sensor/alarm/graph data). Use a filter so
        // ArduinoJson only allocates the few fields we use - without it the parsed document
        // is too big for the ESP32 heap and deserialization fails with NoMemory.
        JsonDocument filter;
        filter["status"] = true;
        filter["data"]["connection"]["glucoseMeasurement"]["ValueInMgPerDl"] = true;
        filter["data"]["connection"]["glucoseMeasurement"]["TrendArrow"] = true;
        filter["data"]["connection"]["glucoseMeasurement"]["Timestamp"] = true;
        filter["data"]["connection"]["glucoseMeasurement"]["FactoryTimestamp"] = true;
        filter["data"]["graphData"][0]["ValueInMgPerDl"] = true;
        filter["data"]["graphData"][0]["Timestamp"] = true; // needed for interval sampling
        
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, response,
                                                     DeserializationOption::Filter(filter));
        
        if (error) {
            Serial.print("LibreView: JSON parse error: ");
            Serial.println(error.c_str());
            return false;
        }
        
        // Check status
        int status = doc["status"] | -1;
        if (status != 0) {
            Serial.printf("LibreView: Glucose data returned status %d\n", status);
            return false;
        }
        
        // Get current glucose from connection.glucoseMeasurement
        JsonObject connection = doc["data"]["connection"];
        JsonObject glucoseMeasurement = connection["glucoseMeasurement"];
        
        if (glucoseMeasurement.isNull()) {
            Serial.println("LibreView: No glucose measurement in response");
            return false;
        }
        
        current_glucose = glucoseMeasurement["ValueInMgPerDl"] | 0;
        int trendArrow = glucoseMeasurement["TrendArrow"] | 3; // Default stable (3 = Stable)
        
        // Copy trend arrow
        strncpy(trend, getTrendArrow(trendArrow), 19);
        trend[19] = '\0';
        
        // Get timestamp
        const char* ts = glucoseMeasurement["Timestamp"] | "";
        strncpy(timestamp, ts, 9);
        timestamp[9] = '\0';
        
        // Get previous glucose from graph data
        JsonArray graphData = doc["data"]["graphData"].as<JsonArray>();
        if (graphData.size() >= 2) {
            // Graph data is in chronological order, get second-to-last
            previous_glucose = graphData[graphData.size() - 2]["ValueInMgPerDl"] | current_glucose;
        } else {
            previous_glucose = current_glucose;
        }
        
        // Optionally populate the graph history from the SAME payload (newest first),
        // so the inline graph can refresh without a second HTTP request.
        //
        // The newest history point is the live glucoseMeasurement (so the graph's right edge
        // matches the big number and moves every reading); the rest are sampled from graphData
        // at the requested interval (1 min = dense recent view, 15 min = wide ~6h view).
        if (history != nullptr && historyCount != nullptr) {
            *historyCount = fillSampledHistory(graphData, history, maxHistory,
                                               intervalMinutes, current_glucose);
        }
        
        // Reading time: parse the measurement's FactoryTimestamp (UTC) so "minutes ago"
        // reflects the true sensor reading age. Fall back to current time if parsing fails
        // or the field is missing, so we never show a bogus reading age.
        const char* factoryTs = glucoseMeasurement["FactoryTimestamp"] | "";
        unsigned long long parsedMs = parseLibreTimestampUtcMs(factoryTs);
        if (parsedMs > 0) {
            readingTime = parsedMs;
        } else {
            Serial.printf("LibreView: could not parse FactoryTimestamp '%s' - using current time\n", factoryTs);
            readingTime = ((unsigned long long)time(nullptr)) * 1000ULL;
        }
        
        Serial.printf("LibreView: Glucose = %.0f, Trend = %s, FactoryTimestamp = %s\n",
                      current_glucose, trend, factoryTs);
        return true;
    }
    
    // Fetch glucose history for graph
    bool fetchGlucoseHistory(GlucoseReading* history, int& count, int maxCount = 24,
                             int intervalMinutes = 1, int attempt = 0) {
        if (!isAuthenticated || strlen(patientId) == 0) {
            Serial.println("LibreView: Not ready to fetch history");
            return false;
        }
        
        WiFiClientSecure client;
        client.setInsecure();
        
        HTTPClient http;
        char endpoint[128];
        snprintf(endpoint, sizeof(endpoint), LIBRE_GRAPH_ENDPOINT, patientId);
        String url = String(baseUrl) + endpoint;
        
        if (!http.begin(client, url)) {
            return false;
        }
        
        http.setTimeout(LIBRE_HTTP_TIMEOUT_MS);
        addHeaders(http, true);
        
        int httpCode = http.GET();
        lastHttpError = httpCode;
        
        if (httpCode != 200) {
            String body = http.getString();
            Serial.printf("LibreView: Get history failed with code %d\n", httpCode);
            Serial.println(body); // log body (often reveals e.g. minimumVersion)
            http.end();
            // Self-heal: if Abbott rejected our version, adopt the required one and retry once.
            if (attempt == 0 && adoptRequiredVersion(body)) {
                return fetchGlucoseHistory(history, count, maxCount, intervalMinutes, attempt + 1);
            }
            return false;
        }
        
        // getString() de-chunks the response; filter keeps the parsed document small enough
        // for the ESP32 heap (see fetchGlucoseData for details).
        String response = http.getString();
        http.end();
        
        JsonDocument filter;
        filter["data"]["graphData"][0]["ValueInMgPerDl"] = true;
        filter["data"]["graphData"][0]["Timestamp"] = true; // needed for interval sampling
        
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, response,
                                                     DeserializationOption::Filter(filter));
        
        if (error) {
            return false;
        }
        
        JsonArray graphData = doc["data"]["graphData"].as<JsonArray>();
        
        // No live measurement available in this standalone fetch, so the newest dot is the
        // newest graph point (liveValue = -1). The periodic fetchGlucoseData() refresh will
        // replace it with the live value on the next reading.
        count = fillSampledHistory(graphData, history, maxCount, intervalMinutes, -1.0f);
        
        Serial.printf("LibreView: Loaded %d history readings (interval %d min)\n", count, intervalMinutes);
        return count > 0;
    }
    
    // Full authentication flow
    bool authenticate() {
        if (!login()) {
            return false;
        }
        if (!getConnections()) {
            return false;
        }
        return true;
    }
    
    // True once the JWT is within 60s of (or past) its expiry, so we can refresh it
    // proactively instead of waiting for a fetch to fail with 401.
    bool isTokenExpired() const {
        if (tokenExpires == 0) return false;        // expiry unknown - let a failed fetch decide
        time_t now = time(nullptr);
        if (now < 24 * 3600) return false;          // clock not yet synced - don't force a false expiry
        return ((unsigned long)now + 60UL) >= tokenExpires;
    }
    
    // Check if client is ready (authenticated, has a patient, and token not expired)
    bool isReady() const {
        return isAuthenticated && strlen(patientId) > 0 && !isTokenExpired();
    }
    
    // HTTP status of the most recent request (e.g. 429 = rate limited). 0 if none/transport error.
    int getLastHttpError() const { return lastHttpError; }
    
    // Check if credentials are configured
    bool hasCredentials() const {
        return strlen(settings.libreViewEmail) > 0 && strlen(settings.libreViewPassword) > 0;
    }
};

#endif // LIBREVIEW_CLIENT_H

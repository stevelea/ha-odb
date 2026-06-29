/*
 * Fork of adlerre/obd2-mqtt — WiFi-only, standard ESP32
 * Stripped: cellular, GPS, deep-sleep, WebSocket
 * Kept: Bluetooth Classic (SPP), MQTT, Web UI config, OTA
 */
#include "mqtt.h"

#include <WiFi.h>
#include <atomic>

#ifndef BUILD_GIT_BRANCH
#define BUILD_GIT_BRANCH ""
#endif
#ifndef BUILD_GIT_COMMIT_HASH
#define BUILD_GIT_COMMIT_HASH ""
#endif

#include <LittleFS.h>

#define FORMAT_LITTLEFS_IF_FAILED true

#define DISCOVERED_DEVICES_FILE "/discovered_devices.json"

#define HA_T_CPUTEMP            "cpuTemp"
#define HA_T_FREEMEM            "freeMem"
#define HA_T_UPTIME             "uptime"
#define HA_T_RECONNECTS         "reconnects"
#define HA_T_IP_ADDR            "ipAddress"
#define HAT_T_DTC               "dtc"
#define HAT_T_CLEAR_DTC         "clearDTC"

#include <numeric>

#include "settings.h"
#include "helper.h"
#include "obd.h"
#include "http.h"

HTTPServer server(80);

#define DEBUG_PORT Serial

MQTT mqtt = MQTT();

std::atomic_bool wifiAPStarted{false};
std::atomic_bool wifiAPInUse{false};
std::atomic<unsigned int> wifiAPStaConnected{0};
std::atomic_bool wifiClientConnected{false};

std::atomic_bool obdConnected{false};
std::atomic<int> obdConnectErrors{0};

std::atomic<unsigned long> startTime{0};

std::atomic_bool allDiscoverySend{false};
std::atomic_bool allDiagnosticDiscoverySend{false};
std::atomic_bool allStaticDiagnosticDiscoverySend{false};

std::atomic<unsigned long> lastDebugOutput{0};
std::atomic<unsigned long> lastMQTTDiscoveryOutput{0};
std::atomic<unsigned long> lastMQTTDiagnosticDiscoveryOutput{0};
std::atomic<unsigned long> lastMQTTStaticDiagnosticDiscoveryOutput{0};
std::atomic<unsigned long> lastMQTTOutput{0};
std::atomic<unsigned long> lastMQTTDiagnosticOutput{0};
std::atomic<unsigned long> lastMQTTStaticDiagnosticOutput{0};
std::atomic<unsigned long> lastMQTTDTCDiagnosticOutput{0};

std::atomic_bool clearDTC{false};

TaskHandle_t outputTaskHdl;
TaskHandle_t stateTaskHdl;

size_t getESPHeapSize() {
    return heap_caps_get_free_size(MALLOC_CAP_8BIT);
}

std::string buildDTCPayload(DTCs *dtcs) {
    JsonDocument doc;
    JsonArray a = doc["dtc"].to<JsonArray>();
    for (int i = 0; i < dtcs->getCount(); ++i) {
        a.add(dtcs->getCode(i)->c_str());
    }
    std::string payload;
    serializeJson(doc, payload);
    return payload;
}

void WiFiAPStart(WiFiEvent_t event, WiFiEventInfo_t info) {
    wifiAPStarted = true;
    DEBUG_PORT.println("AP started.");
}

void WiFiAPStop(WiFiEvent_t event, WiFiEventInfo_t info) {
    wifiAPStarted = false;
    wifiAPInUse = false;
    wifiAPStaConnected = 0;
}

void WiFiAPStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    ++wifiAPStaConnected;
    wifiAPInUse = true;
}

void WiFiAPStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (wifiAPStaConnected != 0) --wifiAPStaConnected;
    if (wifiAPStaConnected == 0) {
        wifiAPInUse = false;
        // Reconnect OBD after AP usage
        OBD.begin(Settings.OBD2.getName("OBDII"), Settings.OBD2.getMAC(), Settings.OBD2.getProtocol(),
                  Settings.OBD2.getCheckPIDSupport(), Settings.OBD2.getDebug(), Settings.OBD2.getSpecifyNumResponses());
        OBD.connect(true);
    }
}

void startWiFiAP() {
    DEBUG_PORT.print("Start AP...");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);

    WiFi.onEvent(WiFiAPStart, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_START);
    WiFi.onEvent(WiFiAPStop, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STOP);
    WiFi.onEvent(WiFiAPStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STACONNECTED);
    WiFi.onEvent(WiFiAPStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);

    String ssid = Settings.WiFi.getAPSSID();
    if (ssid.isEmpty()) {
        ssid = "OBD2-MQTT-" + String(stripChars(WiFi.macAddress().c_str()).c_str());
        Settings.WiFi.setAPSSID(ssid.c_str());
    }
    WiFi.softAP(ssid.c_str(), Settings.WiFi.getAPPassword());
    DEBUG_PORT.println("done.");
}

bool connectWiFi() {
    const char* ssid = Settings.WiFi.getSSID();
    const char* pass = Settings.WiFi.getPassword();
    if (!ssid || strlen(ssid) == 0) return false;

    DEBUG_PORT.printf("Connecting to WiFi: %s...", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        DEBUG_PORT.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiClientConnected = true;
        DEBUG_PORT.printf("\nWiFi connected. IP: %s\n", WiFi.localIP().toString().c_str());
        return true;
    }
    DEBUG_PORT.println("\nWiFi connect failed, falling back to AP mode.");
    return false;
}

void startHttpServer() {
    server.on("/api/version", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, MIME_TYPE_PLAIN, getVersion());
    });

    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, MIME_TYPE_JSON, Settings.buildJson().c_str());
    });

    server.on("/api/settings", HTTP_PUT,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            if (request->contentType() == MIME_TYPE_JSON) {
                if (!index) request->_tempObject = malloc(total);
                if (request->_tempObject != nullptr) {
                    memcpy(static_cast<uint8_t *>(request->_tempObject) + index, data, len);
                    if (index + len == total) {
                        auto json = std::string(static_cast<const char *>(request->_tempObject), total);
                        free(request->_tempObject);
                        if (Settings.parseJson(json)) {
                            Settings.writeSettings(LittleFS);
                            request->send(200);
                        } else {
                            request->send(400);
                        }
                    }
                }
            }
        });

    server.on("/api/obd", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, MIME_TYPE_JSON, OBD.buildJSON().c_str());
    });

    server.on("/api/obd", HTTP_PUT,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            if (request->contentType() == MIME_TYPE_JSON) {
                if (!index) request->_tempObject = malloc(total);
                if (request->_tempObject != nullptr) {
                    memcpy(static_cast<uint8_t *>(request->_tempObject) + index, data, len);
                    if (index + len == total) {
                        auto json = std::string(static_cast<const char *>(request->_tempObject), total);
                        free(request->_tempObject);
                        if (OBD.parseJSON(json)) {
                            OBD.writeStates(LittleFS);
                            request->send(200);
                        } else {
                            request->send(400);
                        }
                    }
                }
            }
        });

    server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200);
        delay(500);
        ESP.restart();
    });

    server.init(LittleFS);
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET,PUT,POST");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "content-type");

    server.begin(LittleFS);
}

void onOBDConnected() {
    obdConnected = true;
    obdConnectErrors = 0;
}

void onOBDConnectError() {
    obdConnected = false;
    ++obdConnectErrors;
}

// ── forward declarations ────────────────────────────────────────────
void publishDiscovery(const char *identifier);

// ── MQTT output task ──────────────────────────────────────────────────

void outputTask(void *parameter) {
    auto identifier = (char *) parameter;

    for (;;) {
        if (wifiClientConnected && !wifiAPInUse) {
            mqtt.loop();

            // Publish diagnostic data every 60s
            auto now = millis();
            if (now - lastDebugOutput > 60000) {
                lastDebugOutput = now;

                if (!allDiscoverySend || !allDiagnosticDiscoverySend || !allStaticDiagnosticDiscoverySend) {
                    publishDiscovery(identifier);
                }

                mqtt.sendTopicUpdate(HA_T_CPUTEMP, std::string(String(temperatureRead(), 1).c_str()));
                mqtt.sendTopicUpdate(HA_T_FREEMEM, std::string(String(getESPHeapSize()).c_str()));
                mqtt.sendTopicUpdate(HA_T_UPTIME, std::string(String((millis() - startTime.load()) / 1000).c_str()));
                mqtt.sendTopicUpdate(HA_T_RECONNECTS, std::string(String(obdConnectErrors.load()).c_str()));
                mqtt.sendTopicUpdate(HA_T_IP_ADDR, std::string(WiFi.localIP().toString().c_str()));
            }

            // Publish OBD data (auto-published via OBD class internally)
            if (obdConnected && !wifiAPInUse) {
                OBD.loop();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void startOutputTask(const char *identifier) {
    if (outputTaskHdl != nullptr) vTaskDelete(outputTaskHdl);
    xTaskCreatePinnedToCore(outputTask, "outputTask", 8192, (void *) identifier, 1, &outputTaskHdl, 0);
}

// ── OBD state read task ───────────────────────────────────────────────

void stateReadTask(void *parameter) {
    for (;;) {
        if (obdConnected && !wifiAPInUse) {
            OBD.readStates(LittleFS);
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void startReadTask() {
    if (stateTaskHdl != nullptr) vTaskDelete(stateTaskHdl);
    xTaskCreatePinnedToCore(stateReadTask, "stateTask", 4096, nullptr, 1, &stateTaskHdl, 1);
}

// ── MQTT Discovery ────────────────────────────────────────────────────

void publishDiscovery(const char *identifier) {
    // Diagnostic sensors
    mqtt.sendTopicConfig(HA_T_CPUTEMP, "CPU Temperature", "thermometer", "°C", "temperature", "measurement", "");
    mqtt.sendTopicConfig(HA_T_FREEMEM, "Free Memory", "memory", "B", "", "measurement", "");
    mqtt.sendTopicConfig(HA_T_UPTIME, "Uptime", "timer-outline", "s", "duration", "total_increasing", "");
    mqtt.sendTopicConfig(HA_T_RECONNECTS, "OBD Reconnects", "connection", "", "", "total_increasing", "");
    mqtt.sendTopicConfig(HA_T_IP_ADDR, "IP Address", "ip-network", "", "", "", "");
    allDiscoverySend = true;
}

// ── Helpers ────────────────────────────────────────────────────────────

String buildIdentifier(const char *devMac) {
    String mID;
    if (Settings.MQTT.getIdType() == MQTTSettings::MQTTIdentifierType::CUSTOM &&
        Settings.MQTT.getIdSuffix().length() > 0) {
        mID = Settings.MQTT.getIdSuffix().c_str();
    } else if (devMac != nullptr && strlen(devMac) > 0) {
        mID = devMac;
    }
    return mID;
}

// ── Setup / Loop ──────────────────────────────────────────────────────

void setup() {
    startTime = millis();
    DEBUG_PORT.begin(115200);

    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
        DEBUG_PORT.println("LittleFS Mount Failed");
        return;
    }

    Settings.readSettings(LittleFS);
    OBD.readStates(LittleFS);

    disableCore0WDT();

    // Try WiFi client mode first, fall back to AP
    if (!connectWiFi()) {
        startWiFiAP();
    }
    startHttpServer();

    OBD.onConnected(onOBDConnected);
    OBD.onConnectError(onOBDConnectError);
    OBD.begin(Settings.OBD2.getName("OBDII"), Settings.OBD2.getMAC(), Settings.OBD2.getProtocol(),
              Settings.OBD2.getCheckPIDSupport(), Settings.OBD2.getDebug(), Settings.OBD2.getSpecifyNumResponses());

    String mID = buildIdentifier(Settings.OBD2.getMAC().c_str());
    if (!mID.isEmpty()) {
        startOutputTask(mID.c_str());
        startReadTask();
    } else {
        startReadTask();
        mID = buildIdentifier(OBD.getConnectedBTAddress().c_str());
        startOutputTask(mID.c_str());
    }
}

void loop() {
    vTaskDelete(nullptr);
}

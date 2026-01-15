#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include "time.h"
#include <esp_task_wdt.h>

// --- OTA Libraries ---
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Update.h>

// --- Modbus Libraries ---
#include <ModbusMaster.h> // RS485 Master
#include <ModbusTCP.h>    // TCP Server
#include <vector>
#include <numeric>
#include <cmath>
#include <algorithm>
#include <map>

// --- FreeRTOS ---
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Global Mutex & Flags
SemaphoreHandle_t dataMutex;
bool shouldReboot = false; // Safe reboot flag

// --- Hardware Config (ESP32-S3-DevKitC-1) ---
// Onboard RGB LED is typically GPIO 48 on DevKitC-1
#define RGB_PIN 48        
#define NUM_PIXELS 1
Adafruit_NeoPixel pixels(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// External Buzzer (Connect to Pin 5)
#define BUZZER_PIN 5

// User Button (Using BOOT Button = GPIO 0)
// Pressing BOOT pulls this pin LOW.
#define USER_BUTTON_PIN 0

// --- MAX485 PIN CONFIGURATION (Moved to UART1) ---
// We cannot use 43/44 on DevKitC-1 as they are used by the USB-Serial Bridge.
// Using standard UART1 pins: TX=17, RX=18.
#define RXD_PIN        18 // Connect to RO (Receiver Output)
#define TXD_PIN        17 // Connect to DI (Driver Input)
#define RS485_CTRL_PIN 14 // Connect to DE and RE

// --- WiFi AP Defaults ---
#define WIFI_AP_SSID "ILA_Sensor_AP"
#define WIFI_AP_PASS "12345678"

// --- Device Info (Dynamic) ---
String sensor_model = "Wilcoxon 883M";
String sensor_serial = "Unknown";
String sensor_vendor_url = "Unknown";
String sensor_part_number = "Unknown";
String sensor_fw_version = "Unknown";
const char* FIRMWARE_VERSION = "4.7.0 - S3-DevKitC-1";

// --- NTP & Time Config ---
const char* ntpServer = "pool.ntp.org";
long  gmtOffset_sec = 28800; // Default GMT+8
int   daylightOffset_sec = 0;

// --- MQTT Configuration ---
bool mqttEnabled = true;
bool mqtt_use_tls = true;
bool mqtt_skip_cert_validation = true;
bool mqtt_publish_progress = true;
String mqtt_server_str = ""; 
int mqtt_port_val = 8883;
String mqtt_user_str = "";
String mqtt_pass_str = "";
String mqtt_topic_str = "883M";
String mqtt_command_topic = "883M/command";
String mqtt_status_topic = "883M/status";
char mqttStatus[128] = "Waiting for config...";

// --- Wi-Fi & Web Server ---
WebServer server(80);
Preferences preferences;
WiFiClient espClient; 
WiFiClientSecure espClientSecure; 
PubSubClient* mqttClient = nullptr; 

// --- Modbus RTU (RS485) ---
uint8_t modbus_slave_id_val = 1;
long modbus_baud_rate_val = 115200;
ModbusMaster node;

// --- Modbus TCP Server ---
bool modbusTcpEnabled = true;
uint16_t modbusTcpPort = 502;
ModbusTCP modbusTCPServer;

// --- LINEAR MODBUS MAP ---
#define HREG_DATA_TYPE_SELECT 80        
#define HREG_WAVEFORM_READY_STATUS 90
#define HREG_SPECTRUM_READY_STATUS 91
#define HREG_TRIGGER_WAVEFORM 92
#define HREG_TRIGGER_SPECTRUM 93

#define HREG_WAVEFORM_START 1000        // Registers 1000 - 14333
#define HREG_SPECTRUM_START 20000       // Registers 20000 - 26144

// --- Modbus Addresses (Sensor Side) ---
#define REG_TRIGGER             0x0200 
#define VAL_TRIGGER_START       0xFFFF 
#define VAL_STATUS_READY        0x0000 
#define VAL_STATUS_BUSY         0xEEEE 

#define ADDR_WAVE_X_START       0x1E01
#define ADDR_WAVE_Y_START       0x7101
#define ADDR_WAVE_Z_START       0xC401
#define ADDR_SPEC_X_START       0x0600
#define ADDR_SPEC_Y_START       0x5900
#define ADDR_SPEC_Z_START       0xAC00

// --- Global State ---
char systemStatus[128] = "Initializing...";
char trafficLightStatus[20] = "IDLE"; 
String globalLogBuffer = ""; 
String lastResetReason = "Unknown";
enum DynamicDataState { DD_IDLE, DD_REQUEST_DATA, DD_WAITING_FOR_DATA };
DynamicDataState currentDataState = DD_IDLE;
enum CaptureMode { CAPTURE_NONE, CAPTURE_WAVEFORM, CAPTURE_SPECTRUM, CAPTURE_BOTH };
CaptureMode requestedCapture = CAPTURE_NONE;
bool isReadingWaveform = false;
int waveformReadIndex = 0;
bool isReadingSpectrum = false;
int spectrumReadIndex = 0;
enum Axis { AXIS_NONE, AXIS_X, AXIS_Y, AXIS_Z };
Axis requestedAxis = AXIS_NONE;
Axis lastCapturedAxis = AXIS_NONE;
char lastCaptureTimestamp[21] = {0};
char lastAutoCaptureTimestamp[21] = "Never";
int autoCaptureAxisIndex = 0;
bool isAutoCaptureCycleActive = false;

// --- Button/Alarm ---
unsigned long buttonPressStartTime = 0;
bool apModeTriggeredByButton = false;
bool wasWifiConnected = false;
bool wifiDisconnectAlarmActive = false;
unsigned long lastAlarmActionTime = 0;
bool isAlarmBeeping = false;

// --- Global Metrics ---
float accelRmsX=0.0, accelRmsY=0.0, accelRmsZ=0.0;
float accelPeakX=0.0, accelPeakY=0.0, accelPeakZ=0.0;
float accelRms2X=0.0, accelRms2Y=0.0, accelRms2Z=0.0;
float accelPeak2X=0.0, accelPeak2Y=0.0, accelPeak2Z=0.0;
float veloRmsX=0.0, veloRmsY=0.0, veloRmsZ=0.0;
float veloPeakX=0.0, veloPeakY=0.0, veloPeakZ=0.0;
float displRmsX=0.0, displRmsY=0.0, displRmsZ=0.0;
float displPeakX=0.0, displPeakY=0.0, displPeakZ=0.0;
float truePeakX=0.0, truePeakY=0.0, truePeakZ=0.0;
float crestFactorX=0.0, crestFactorY=0.0, crestFactorZ=0.0;
float stdDeviationX=0.0, stdDeviationY=0.0, stdDeviationZ=0.0;
float temperature=0.0;
// Extended Metrics
float accelMetricRmsX=0.0, accelMetricRmsY=0.0, accelMetricRmsZ=0.0;     
float accelMetricPeakX=0.0, accelMetricPeakY=0.0, accelMetricPeakZ=0.0; 
float accelMetricRms2X=0.0, accelMetricRms2Y=0.0, accelMetricRms2Z=0.0; 
float accelMetricPeak2X=0.0, accelMetricPeak2Y=0.0, accelMetricPeak2Z=0.0; 
float veloWideRmsX=0.0, veloWideRmsY=0.0, veloWideRmsZ=0.0;
float veloWidePeakX=0.0, veloWidePeakY=0.0, veloWidePeakZ=0.0;
float displLowRmsX=0.0, displLowRmsY=0.0, displLowRmsZ=0.0;
float displLowPeakX=0.0, displLowPeakY=0.0, displLowPeakZ=0.0;

// --- Buffers ---
const uint16_t WAVEFORM_TOTAL_POINTS = 13334;
int16_t fullWaveform[WAVEFORM_TOTAL_POINTS];
bool newWaveformAvailable = false;
const uint16_t SPECTRUM_TOTAL_POINTS = 6145;
int16_t spectrumData[SPECTRUM_TOTAL_POINTS];
bool newSpectrumAvailable = false;
int16_t globalScratchBuffer[WAVEFORM_TOTAL_POINTS]; 

// --- Spectrum Helpers ---
const float SPECTRUM_SCALE_FACTOR = 0.001; 
const float SPECTRUM_FREQ_RESOLUTION = (13333.333 / 16384.0); 
struct SpectrumPeak { float frequency_hz; float amplitude_g; };
std::map<Axis, std::vector<SpectrumPeak>> xyzPeakDataCache;

// --- Metrics Logger ---
const int METRICS_LOG_SIZE = 30;
const int METRICS_SNAPSHOT_SIZE = 31 + 12; 
struct MetricsSnapshot { char timestamp[21] = {0}; float values[METRICS_SNAPSHOT_SIZE]; bool is_valid = false; };
MetricsSnapshot metricsLog[METRICS_LOG_SIZE];
bool isMetricsRecording = false;
int metricsLogIndex = 0;
int metricsLogCount = 0;

// --- Timers ---
unsigned long lastMetricsReadTime = 0;
long metricsReadInterval_ms = 2000;
unsigned long lastCaptureActionTime = 0;
unsigned long lastAutoCaptureMillis = 0;
long dataCaptureInterval = 600000;
const long INTERVAL_NEVER = 2147483647L;
unsigned long ledBlinkTime = 0;
bool ledState = false;
const long dataPollInterval = 500;
unsigned long lastPollTime = 0;
const int maxPollAttempts = 20;
int pollAttemptCounter = 0;
unsigned long lastMqttAttemptTime = 0;
const long mqttReconnectInterval = 5000;
unsigned long lastHealthPublishTime = 0;
const long healthPublishInterval = 300000; 

// --- Forward Declarations ---
void handleRoot(); void handleMetrics(); void handleFullWaveform(); void handleFullSpectrum(); 
void handleSettings(); void handleWifiSettings(); void handleSaveWifi(); 
void handleMqttSettings(); void handleSaveMqtt();
void handleRs485Settings(); void handleSaveRs485(); 
void handleTriggerCapture(); void handleSetIntervals();
void handleGetIntervals(); void handleScanWifi(); void handleNotFound();
void handleGetMetricsLogData(); void handleMetricsLogControl(); void handleDownloadMetricsCsv();
void handleClearMetricsLog(); void handleClearDynamicData(); void handleReboot();
void handleDeviceInfo(); void handleModbusTcpSettings(); void handleSaveModbusTcp();
void handleSetMetricsInterval(); 
void handleTimeSettings(); void handleSaveTimeSettings(); void handleSetManualTime();
void handleClearSystemLog();
void handleUpdateResult(); 
void handleUpdateUpload();
void handleMqttStatus();

// =================================================================
// --- UTILITY FUNCTIONS
// =================================================================
String formatUptime(unsigned long ms) {
    unsigned long seconds = ms / 1000;
    int days = seconds / 86400;
    seconds %= 86400;
    int hours = seconds / 3600;
    seconds %= 3600;
    int minutes = seconds / 60;
    return String(days) + "d " + String(hours) + "h " + String(minutes) + "m";
}

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) { return "No Time Sync"; }
  char timeString[21];
  strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(timeString);
}

void logMessage(String msg) {
    Serial.println(msg);
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        String ts = getTimestamp();
        if(ts == "No Time Sync") ts = String(millis()/1000) + "s";
        const char* taskName = pcTaskGetName(NULL);
        String newEntry = ts + " [" + String(taskName) + "] " + msg + "\n";
        globalLogBuffer += newEntry;
        if (globalLogBuffer.length() > 3000) {
            globalLogBuffer = globalLogBuffer.substring(globalLogBuffer.length() - 3000);
        }
        xSemaphoreGive(dataMutex);
    }
}

String getResetReasonString() {
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON: return "Power On";
        case ESP_RST_EXT: return "External Pin Reset";
        case ESP_RST_SW: return "Software Reset";
        case ESP_RST_PANIC: return "Crash (Panic/Exception)";
        case ESP_RST_INT_WDT: return "Internal Watchdog";
        case ESP_RST_TASK_WDT: return "Task WDT (Background Loop Stuck)";
        case ESP_RST_WDT: return "Other Watchdog";
        case ESP_RST_DEEPSLEEP: return "Deep Sleep Wake";
        case ESP_RST_BROWNOUT: return "Brownout";
        default: return "Unknown (" + String(reason) + ")";
    }
}

void updateLedStatus() {
    unsigned long currentMillis = millis();
    bool local_isReading = false;
    bool local_isMetricsRecording = false;
    char local_status[128];

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        local_isReading = isReadingWaveform || isReadingSpectrum || currentDataState == DD_WAITING_FOR_DATA;
        local_isMetricsRecording = isMetricsRecording;
        snprintf(local_status, sizeof(local_status), "%s", systemStatus);
        xSemaphoreGive(dataMutex);
    }

    if (local_isReading) {
        if (currentMillis - ledBlinkTime > 500) {
            ledBlinkTime = currentMillis;
            ledState = !ledState;
            pixels.setPixelColor(0, ledState ? pixels.Color(255, 255, 0) : pixels.Color(0, 0, 0)); // Yellow
            pixels.show();
        }
        return;
    }
      if (local_isMetricsRecording) {
        if (currentMillis - ledBlinkTime > 500) {
            ledBlinkTime = currentMillis;
            ledState = !ledState;
            pixels.setPixelColor(0, ledState ? pixels.Color(0, 255, 255) : pixels.Color(0, 0, 0)); // Cyan
            pixels.show();
        }
        return;
    }
    if (strstr(local_status, "failed") != NULL || strstr(local_status, "Error") != NULL || strstr(local_status, "Timeout") != NULL || strstr(local_status, "Disconnected") != NULL) {
        pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red
    } else if (WiFi.status() != WL_CONNECTED) {
        pixels.setPixelColor(0, pixels.Color(200, 200, 200)); // White
    } else {
        pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // Green
    }
    pixels.show();
}

std::vector<SpectrumPeak> findSpectrumPeaks(const int16_t* spectrum_buffer, int num_peaks) {
    std::vector<SpectrumPeak> all_points;
    all_points.reserve(SPECTRUM_TOTAL_POINTS);
    for (int i = 0; i < SPECTRUM_TOTAL_POINTS; i++) {
        all_points.push_back({ .frequency_hz = i * SPECTRUM_FREQ_RESOLUTION, .amplitude_g = abs(spectrum_buffer[i] * SPECTRUM_SCALE_FACTOR) });
    }
    std::sort(all_points.begin(), all_points.end(), [](const SpectrumPeak& a, const SpectrumPeak& b) { return a.amplitude_g > b.amplitude_g; });
    if (all_points.size() > num_peaks) { all_points.resize(num_peaks); }
    std::sort(all_points.begin(), all_points.end(), [](const SpectrumPeak& a, const SpectrumPeak& b) { return a.frequency_hz < b.frequency_hz; });
    return all_points;
}

// --- MAX485 CALLBACKS ---
void preTransmission() {
  digitalWrite(RS485_CTRL_PIN, HIGH); // DE/RE High = Transmit
}

void postTransmission() {
  digitalWrite(RS485_CTRL_PIN, LOW);  // DE/RE Low = Receive
}

// =================================================================
// --- MODBUS IDENTITY FUNCTION
// =================================================================
void fetchSensorIdentity() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) != pdTRUE) return;
    
    logMessage("[System] Fetching Sensor Identity...");

    // Must enable TX manually for raw serial writes
    digitalWrite(RS485_CTRL_PIN, HIGH);
    delay(2);

    uint8_t readCodes[2] = {0x01, 0x02};
    for (int q = 0; q < 2; q++) {
        uint8_t currentCode = readCodes[q];
        while(Serial1.available()) Serial1.read();
        
        uint8_t request[] = {modbus_slave_id_val, 0x2B, 0x0E, currentCode, 0x00, 0x00, 0x00};
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < 5; i++) {
            crc ^= request[i];
            for (int j = 0; j < 8; j++) { if (crc & 1) crc = (crc >> 1) ^ 0xA001; else crc >>= 1; }
        }
        request[5] = crc & 0xFF; request[6] = crc >> 8;

        Serial1.write(request, 7);
        Serial1.flush(); // Wait for TX to finish
        
        // Switch to RX immediately after flush
        digitalWrite(RS485_CTRL_PIN, LOW); 
        delay(300); 

        uint8_t buffer[256]; 
        int idx = 0;
        while(Serial1.available() && idx < 256) { buffer[idx++] = Serial1.read(); }

        if (idx > 8 && buffer[1] == 0x2B && buffer[2] == 0x0E && buffer[3] == currentCode) {
            int objCount = buffer[7]; int ptr = 8; 
            for (int i = 0; i < objCount; i++) {
                if (ptr + 2 >= idx) break; 
                uint8_t objId = buffer[ptr]; uint8_t objLen = buffer[ptr+1];
                String objData = "";
                for (int k = 0; k < objLen; k++) { if (ptr + 2 + k < idx) objData += (char)buffer[ptr + 2 + k]; }
                if (objId == 0x01) sensor_serial = objData;        
                if (objId == 0x02) sensor_fw_version = objData;  
                if (objId == 0x05) sensor_model = objData;        
                if (objId == 0x03) sensor_vendor_url = objData;  
                if (objId == 0x04) sensor_part_number = objData; 
                ptr += (2 + objLen); 
            }
        }
        delay(100); 
        // Re-enable TX for next loop iteration
        digitalWrite(RS485_CTRL_PIN, HIGH);
    }
    
    // Ensure we end in RX mode
    digitalWrite(RS485_CTRL_PIN, LOW);

    logMessage("[System] ID Result -> Model: " + sensor_model + " | Serial: " + sensor_serial + " | PN: " + sensor_part_number);
    xSemaphoreGive(dataMutex);
}

// =================================================================
// --- MQTT FUNCTIONS
// =================================================================
void publishDeviceHealth() {
    if (!mqttClient || !mqttClient->connected()) return;
    DynamicJsonDocument doc(2048); 
    doc["model"] = sensor_model; 
    doc["serial_number"] = sensor_serial; 
    doc["vendor_url"] = sensor_vendor_url; 
    doc["timestamp"] = getTimestamp();
    doc["type"] = "health_status";
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        doc["uptime"] = formatUptime(millis());
        doc["wifi_signal_rssi"] = WiFi.RSSI();
        doc["free_heap_memory"] = ESP.getFreeHeap();
        doc["reset_reason"] = lastResetReason;
        xSemaphoreGive(dataMutex);
    } else { return; }
    char payload[2048];
    size_t n = serializeJson(doc, payload);
    mqttClient->publish(mqtt_status_topic.c_str(), payload, n);
}

void publishMetrics() {
    if (!mqttClient || !mqttClient->connected()) return;
    DynamicJsonDocument doc(4096); 
    doc["model"] = sensor_model;
    doc["serial_number"] = sensor_serial;
    doc["vendor_url"] = sensor_vendor_url; 
    doc["timestamp"] = getTimestamp();
    doc["type"] = "metrics";
    JsonObject data = doc.createNestedObject("data");

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        JsonObject accelRms = data.createNestedObject("accelRms");
        accelRms["x"] = accelRmsX; accelRms["y"] = accelRmsY; accelRms["z"] = accelRmsZ;
        JsonObject accelPeak = data.createNestedObject("accelPeak");
        accelPeak["x"] = accelPeakX; accelPeak["y"] = accelPeakY; accelPeak["z"] = accelPeakZ;
        JsonObject accelRms2 = data.createNestedObject("accelRms2");
        accelRms2["x"] = accelRms2X; accelRms2["y"] = accelRms2Y; accelRms2["z"] = accelRms2Z;
        JsonObject accelPeak2 = data.createNestedObject("accelPeak2");
        accelPeak2["x"] = accelPeak2X; accelPeak2["y"] = accelPeak2Y; accelPeak2["z"] = accelPeak2Z;
        
        JsonObject amRms = data.createNestedObject("accelMetricRms");
        amRms["x"] = accelMetricRmsX; amRms["y"] = accelMetricRmsY; amRms["z"] = accelMetricRmsZ;
        JsonObject amPeak = data.createNestedObject("accelMetricPeak");
        amPeak["x"] = accelMetricPeakX; amPeak["y"] = accelMetricPeakY; amPeak["z"] = accelMetricPeakZ;
        JsonObject amRms2 = data.createNestedObject("accelMetricRms2");
        amRms2["x"] = accelMetricRms2X; amRms2["y"] = accelMetricRms2Y; amRms2["z"] = accelMetricRms2Z;
        JsonObject amPeak2 = data.createNestedObject("accelMetricPeak2");
        amPeak2["x"] = accelMetricPeak2X; amPeak2["y"] = accelMetricPeak2Y; amPeak2["z"] = accelMetricPeak2Z;

        JsonObject veloRms = data.createNestedObject("veloRms");
        veloRms["x"] = veloRmsX; veloRms["y"] = veloRmsY; veloRms["z"] = veloRmsZ;
        JsonObject veloPeak = data.createNestedObject("veloPeak");
        veloPeak["x"] = veloPeakX; veloPeak["y"] = veloPeakY; veloPeak["z"] = veloPeakZ;
        
        JsonObject vwRms = data.createNestedObject("veloWideRms");
        vwRms["x"] = veloWideRmsX; vwRms["y"] = veloWideRmsY; vwRms["z"] = veloWideRmsZ;
        JsonObject vwPeak = data.createNestedObject("veloWidePeak");
        vwPeak["x"] = veloWidePeakX; vwPeak["y"] = veloWidePeakY; vwPeak["z"] = veloWidePeakZ;
        
        JsonObject displRms = data.createNestedObject("displRms");
        displRms["x"] = displRmsX; displRms["y"] = displRmsY; displRms["z"] = displRmsZ;
        JsonObject displPeak = data.createNestedObject("displPeak");
        displPeak["x"] = displPeakX; displPeak["y"] = displPeakY; displPeak["z"] = displPeakZ;
        
        JsonObject dlRms = data.createNestedObject("displLowRms");
        dlRms["x"] = displLowRmsX; dlRms["y"] = displLowRmsY; dlRms["z"] = displLowRmsZ;
        JsonObject dlPeak = data.createNestedObject("displLowPeak");
        dlPeak["x"] = displLowPeakX; dlPeak["y"] = displLowPeakY; dlPeak["z"] = displLowPeakZ;

        JsonObject truePeak = data.createNestedObject("truePeak");
        truePeak["x"] = truePeakX; truePeak["y"] = truePeakY; truePeak["z"] = truePeakZ;
        JsonObject crestFactor = data.createNestedObject("crestFactor");
        crestFactor["x"] = crestFactorX; crestFactor["y"] = crestFactorY; crestFactor["z"] = crestFactorZ;
        JsonObject stdDeviation = data.createNestedObject("stdDeviation");
        stdDeviation["x"] = stdDeviationX; stdDeviation["y"] = stdDeviationY; stdDeviation["z"] = stdDeviationZ;
        
        data["temperature"] = temperature;
        xSemaphoreGive(dataMutex);
    } else { return; }
    char payload[4096];
    size_t n = serializeJson(doc, payload);
    mqttClient->publish(mqtt_topic_str.c_str(), payload, n);
}

void publishDownsampledWaveform() {
  if (!mqttClient || !mqttClient->connected()) return;
  DynamicJsonDocument doc(6144); 
  doc["model"] = sensor_model;
  doc["serial_number"] = sensor_serial;
  doc["timestamp"] = getTimestamp();
  doc["type"] = "waveform_preview";
  JsonArray data = doc.createNestedArray("data");
  const int DOWNSAMPLE_FACTOR = 100;
  const float scaleFactor = 0.001;

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (!newWaveformAvailable) {
        xSemaphoreGive(dataMutex);
        return;
    }
    for (int i = 0; i < WAVEFORM_TOTAL_POINTS; i += DOWNSAMPLE_FACTOR) {
      data.add(fullWaveform[i] * scaleFactor);
    }
    xSemaphoreGive(dataMutex);
  } else { return; }

  char payload[6144];
  size_t n = serializeJson(doc, payload);
  mqttClient->publish(mqtt_topic_str.c_str(), payload, n);
}

void publishFullWaveformInChunks(Axis capturedAxis) {
    if (!mqttClient || !mqttClient->connected()) return;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (!newWaveformAvailable) {
            xSemaphoreGive(dataMutex);
            return;
        }
        memcpy(globalScratchBuffer, fullWaveform, WAVEFORM_TOTAL_POINTS * sizeof(int16_t));
        xSemaphoreGive(dataMutex);
    } else {
        return;
    }

    const int POINTS_PER_CHUNK = 600;
    const float scaleFactor = 0.001;
    String captureId = String(millis());
    int totalChunks = (WAVEFORM_TOTAL_POINTS + POINTS_PER_CHUNK - 1) / POINTS_PER_CHUNK;
    String axisStr = "N/A";
    switch(capturedAxis) {
        case AXIS_X: axisStr = "X"; break;
        case AXIS_Y: axisStr = "Y"; break;
        case AXIS_Z: axisStr = "Z"; break;
        default: break;
    }

    for (int i = 0; i < totalChunks; i++) {
        esp_task_wdt_reset(); 
        
        DynamicJsonDocument doc(4096); 
        doc["model"] = sensor_model;
        doc["serial_number"] = sensor_serial;
        doc["timestamp"] = getTimestamp();
        doc["type"] = "waveform_chunk";
        doc["axis"] = axisStr;
        doc["capture_id"] = captureId;
        doc["chunk_index"] = i;
        doc["total_chunks"] = totalChunks;
        JsonArray data = doc.createNestedArray("data");
        int startIndex = i * POINTS_PER_CHUNK;
        int endIndex = min(startIndex + POINTS_PER_CHUNK, (int)WAVEFORM_TOTAL_POINTS);
        for (int j = startIndex; j < endIndex; j++) {
            data.add(globalScratchBuffer[j] * scaleFactor);
        }
        char payload[4096];
        size_t n = serializeJson(doc, payload);
        mqttClient->publish(mqtt_topic_str.c_str(), payload, n);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void publishFullSpectrumInChunks(Axis capturedAxis) {
    if (!mqttClient || !mqttClient->connected()) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (!newSpectrumAvailable) {
            xSemaphoreGive(dataMutex);
            return;
        }
        memcpy(globalScratchBuffer, spectrumData, SPECTRUM_TOTAL_POINTS * sizeof(int16_t));
        xSemaphoreGive(dataMutex);
    } else {
        return;
    }

    const int POINTS_PER_CHUNK = 600;
    const float scaleFactor = 0.001;
    String captureId = String(millis());
    int totalChunks = (SPECTRUM_TOTAL_POINTS + POINTS_PER_CHUNK - 1) / POINTS_PER_CHUNK;
    String axisStr = "N/A";
    switch(capturedAxis) {
        case AXIS_X: axisStr = "X"; break;
        case AXIS_Y: axisStr = "Y"; break;
        case AXIS_Z: axisStr = "Z"; break;
        default: break;
    }

    for (int i = 0; i < totalChunks; i++) {
        esp_task_wdt_reset(); 
        
        DynamicJsonDocument doc(4096); 
        doc["model"] = sensor_model;
        doc["serial_number"] = sensor_serial;
        doc["timestamp"] = getTimestamp();
        doc["type"] = "spectrum_chunk";
        doc["axis"] = axisStr;
        doc["capture_id"] = captureId;
        doc["chunk_index"] = i;
        doc["total_chunks"] = totalChunks;
        JsonArray data = doc.createNestedArray("data");
        int startIndex = i * POINTS_PER_CHUNK;
        int endIndex = min(startIndex + POINTS_PER_CHUNK, (int)SPECTRUM_TOTAL_POINTS);
        for (int j = startIndex; j < endIndex; j++) {
            data.add(abs(globalScratchBuffer[j] * scaleFactor));
        }
        char payload[4096];
        size_t n = serializeJson(doc, payload);
        mqttClient->publish(mqtt_topic_str.c_str(), payload, n);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void publishSpectrumPeaks(Axis capturedAxis, const std::vector<SpectrumPeak>& peaks) {
    if (!mqttClient || !mqttClient->connected()) return;
    DynamicJsonDocument doc(2048);
    doc["model"] = sensor_model;
    doc["serial_number"] = sensor_serial;
    doc["timestamp"] = getTimestamp();
    doc["type"] = "spectrum_peaks";
    String axisStr = "N/A";
    switch(capturedAxis) {
        case AXIS_X: axisStr = "X"; break;
        case AXIS_Y: axisStr = "Y"; break;
        case AXIS_Z: axisStr = "Z"; break;
        default: break;
    }
    doc["axis"] = axisStr;
    JsonArray peaks_array = doc.createNestedArray("peaks");
    for (const auto& peak : peaks) {
        JsonObject p = peaks_array.createNestedObject();
        p["freq_hz"] = round(peak.frequency_hz * 100) / 100.0; 
        p["amp_g"] = round(peak.amplitude_g * 10000) / 10000.0; 
    }
    char payload[2048];
    size_t n = serializeJson(doc, payload);
    mqttClient->publish(mqtt_topic_str.c_str(), payload, n);
}

void publishCombinedSpectrumPeaks() {
    if (!mqttClient || !mqttClient->connected() || xyzPeakDataCache.empty()) return;
    DynamicJsonDocument doc(4096);
    doc["model"] = sensor_model;
    doc["serial_number"] = sensor_serial;
    doc["timestamp"] = getTimestamp();
    doc["type"] = "combined_spectrum_peaks";
    JsonObject peaks_obj = doc.createNestedObject("peaks");
    for (auto const& [axis, peaks] : xyzPeakDataCache) {
        String axisStr = (axis == AXIS_X) ? "X" : (axis == AXIS_Y) ? "Y" : "Z";
        JsonArray peaks_array = peaks_obj.createNestedArray(axisStr);
        for (const auto& peak : peaks) {
            JsonObject p = peaks_array.createNestedObject();
            p["freq_hz"] = round(peak.frequency_hz * 100) / 100.0;
            p["amp_g"] = round(peak.amplitude_g * 10000) / 10000.0;
        }
    }
    char payload[4096];
    size_t n = serializeJson(doc, payload);
    mqttClient->publish(mqtt_status_topic.c_str(), payload, n);
    xyzPeakDataCache.clear();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String topic_str(topic);
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload, length);
    
    const char* target_sn = doc["serial_number"];
    
    // --- STRICT SERIAL NUMBER MATCHING LOGIC ---
    if (target_sn && String(target_sn) == sensor_serial) {
        
        const char* command = doc["command"];
        if (command) {
            logMessage("MQTT Command received: " + String(command));
            
            if (strcmp(command, "get_metrics") == 0) {
                publishMetrics();
                return;
            }
            
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                if (strcmp(command, "trigger_all") == 0) {
                    requestedAxis = AXIS_X;                 
                    requestedCapture = CAPTURE_BOTH;        
                    currentDataState = DD_REQUEST_DATA; 
                    
                    isAutoCaptureCycleActive = true;        
                    autoCaptureAxisIndex = 0;               
                    xyzPeakDataCache.clear();               
                    
                    logMessage("MQTT: Starting Full XYZ Capture Sequence");
                }
                else if (strcmp(command, "trigger_capture") == 0) {
                    String axis = doc["axis"] | "X";
                    if(axis == "X") requestedAxis = AXIS_X;
                    else if(axis == "Y") requestedAxis = AXIS_Y;
                    else if(axis == "Z") requestedAxis = AXIS_Z;
                    
                    if(requestedAxis != AXIS_NONE) {
                        requestedCapture = CAPTURE_BOTH;
                        currentDataState = DD_REQUEST_DATA;
                        isAutoCaptureCycleActive = false; 
                        logMessage("MQTT: Triggering Single Axis " + axis);
                    }
                } 
                else if (strcmp(command, "set_interval") == 0) {
                    long newInterval = doc["interval_ms"];
                    if (newInterval > 0) {
                        dataCaptureInterval = newInterval;
                        preferences.begin("device-cfg", false);
                        preferences.putLong("wf_interval", dataCaptureInterval);
                        preferences.end();
                        logMessage("Interval updated via MQTT to " + String(newInterval));
                    }
                } 
                xSemaphoreGive(dataMutex);
            }
            if (strcmp(command, "reboot") == 0) { 
                logMessage("Remote reboot requested. Restarting..."); 
                shouldReboot = true; 
            } 
        }
    } else {
        String receivedSN = target_sn ? String(target_sn) : "MISSING";
        logMessage("[MQTT] Command IGNORED. SN mismatch. Received: " + receivedSN + ", Device: " + sensor_serial);
    }
}

// =================================================================
// --- MODBUS SENSOR FUNCTIONS
// =================================================================
bool requestNewWaveform() {
    uint8_t result = node.writeSingleRegister(REG_TRIGGER, VAL_TRIGGER_START);
    return (result == node.ku8MBSuccess);
}

bool isWaveformReady() {
    uint8_t result = node.readHoldingRegisters(REG_TRIGGER, 1);
    if (result == node.ku8MBSuccess) {
        return (node.getResponseBuffer(0) == VAL_STATUS_READY);
    }
    return false;
}

bool readSensorValue(const char* name, uint16_t address, float scale, float &target) {
    uint8_t result = node.readInputRegisters(address, 1);
    if (result == node.ku8MBSuccess) {
        target = (int16_t)node.getResponseBuffer(0) / scale;
        return true;
    }
    return false;
}

bool readLargeDataBlock(uint16_t startAddress, uint16_t totalPoints, int16_t* buffer) {
    uint16_t pointsRead = 0;
    const uint16_t chunkSize = 125;
    int retryCount = 0;
    const int maxRetries = 3;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        snprintf(systemStatus, sizeof(systemStatus), "Downloading... (0%%)");
        strcpy(trafficLightStatus, "BUSY");
        xSemaphoreGive(dataMutex);
    }

    while (pointsRead < totalPoints) {
        esp_task_wdt_reset(); 

        if (modbusTcpEnabled) {
             modbusTCPServer.task();
        }

        uint16_t pointsLeft = totalPoints - pointsRead;
        uint16_t pointsToRead = (pointsLeft > chunkSize) ? chunkSize : pointsLeft;
        uint8_t result = node.readInputRegisters(startAddress + pointsRead, pointsToRead);

        if (result == node.ku8MBSuccess) {
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                for (int i = 0; i < pointsToRead; i++) {
                    buffer[pointsRead + i] = (int16_t)node.getResponseBuffer(i);
                }
                xSemaphoreGive(dataMutex);
            }
            
            pointsRead += pointsToRead;
            retryCount = 0;

            if (pointsRead % 1000 < chunkSize || pointsRead == totalPoints) {
                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    snprintf(systemStatus, sizeof(systemStatus), "Downloading... (%d%%)", (pointsRead * 100) / totalPoints);
                    xSemaphoreGive(dataMutex);
                }
                if (mqttClient && mqttClient->connected()) mqttClient->loop(); 
                esp_task_wdt_reset(); 
            }
        } else {
            retryCount++;
            if (retryCount > maxRetries) {
                logMessage("[Modbus] Read fail (Addr: " + String(startAddress + pointsRead) + ")");
                return false; 
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return true;
}

void logMetrics() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        MetricsSnapshot& entry = metricsLog[metricsLogIndex];
        strncpy(entry.timestamp, getTimestamp().c_str(), 20);
        
        entry.values[0] = accelRmsX; entry.values[1] = accelRmsY; entry.values[2] = accelRmsZ;
        entry.values[3] = accelPeakX; entry.values[4] = accelPeakY; entry.values[5] = accelPeakZ;
        entry.values[6] = veloRmsX; entry.values[7] = veloRmsY; entry.values[8] = veloRmsZ;
        entry.values[9] = veloPeakX; entry.values[10] = veloPeakY; entry.values[11] = veloPeakZ;
        entry.values[12] = displRmsX; entry.values[13] = displRmsY; entry.values[14] = displRmsZ;
        entry.values[15] = displPeakX; entry.values[16] = displPeakY; entry.values[17] = displPeakZ;
        entry.values[18] = truePeakX; entry.values[19] = truePeakY; entry.values[20] = truePeakZ;
        entry.values[21] = crestFactorX; entry.values[22] = crestFactorY; entry.values[23] = crestFactorZ;
        entry.values[24] = stdDeviationX; entry.values[25] = stdDeviationY; entry.values[26] = stdDeviationZ;
        entry.values[27] = accelRms2X; entry.values[28] = accelRms2Y; entry.values[29] = accelRms2Z;
        entry.values[30] = temperature;
        entry.values[31] = accelMetricRmsX; entry.values[32] = accelMetricRmsY; entry.values[33] = accelMetricRmsZ;
        entry.values[34] = accelMetricPeakX; entry.values[35] = accelMetricPeakY; entry.values[36] = accelMetricPeakZ;
        entry.values[37] = accelMetricRms2X; entry.values[38] = accelMetricRms2Y; entry.values[39] = accelMetricRms2Z;
        entry.values[40] = accelMetricPeak2X; entry.values[41] = accelMetricPeak2Y; entry.values[42] = accelMetricPeak2Z;
        
        entry.is_valid = true;
        
        metricsLogIndex = (metricsLogIndex + 1) % METRICS_LOG_SIZE;
        if (metricsLogCount < METRICS_LOG_SIZE) metricsLogCount++;
        
        xSemaphoreGive(dataMutex);
    }
}

void handleLongReads() {
    bool local_isReadingWaveform, local_isReadingSpectrum;
    Axis local_requestedAxis;
    bool local_isAutoCaptureCycleActive = false;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        local_isReadingWaveform = isReadingWaveform;
        local_isReadingSpectrum = isReadingSpectrum;
        local_requestedAxis = requestedAxis;
        local_isAutoCaptureCycleActive = isAutoCaptureCycleActive;
        xSemaphoreGive(dataMutex);
    } else { return; }

    char axisChar = (local_requestedAxis == AXIS_X) ? 'X' : (local_requestedAxis == AXIS_Y ? 'Y' : 'Z');

    if (local_isReadingWaveform) {
        uint16_t START_ADDRESS = 0;
        if (local_requestedAxis == AXIS_X) START_ADDRESS = ADDR_WAVE_X_START;
        else if (local_requestedAxis == AXIS_Y) START_ADDRESS = ADDR_WAVE_Y_START;
        else if (local_requestedAxis == AXIS_Z) START_ADDRESS = ADDR_WAVE_Z_START;
        else { 
            logMessage("[Waveform] Capture failed: Invalid Axis.");
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                isReadingWaveform = false;
                xSemaphoreGive(dataMutex);
            }
            return; 
        }

        bool success = readLargeDataBlock(START_ADDRESS, WAVEFORM_TOTAL_POINTS, fullWaveform);

        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            isReadingWaveform = false; 
            if (success) {
                newWaveformAvailable = true;
                isReadingSpectrum = true; 
                
                // --- UPDATE MODBUS STATUS FOR PLC ---
                if(modbusTcpEnabled) {
                    uint16_t statusVal = 0;
                    if(local_requestedAxis == AXIS_X) statusVal = 2;
                    else if(local_requestedAxis == AXIS_Y) statusVal = 3;
                    else if(local_requestedAxis == AXIS_Z) statusVal = 4;
                    modbusTCPServer.Hreg(HREG_WAVEFORM_READY_STATUS, statusVal);
                }
                
                snprintf(systemStatus, sizeof(systemStatus), "Waveform (%c) Done. Reading Spectrum...", axisChar);
            } else {
                snprintf(systemStatus, sizeof(systemStatus), "Waveform Download Failed");
                strcpy(trafficLightStatus, "ERROR");
                currentDataState = DD_IDLE; 
                isAutoCaptureCycleActive = false;
            }
            xSemaphoreGive(dataMutex);
        }
        if (success && mqttClient && mqttClient->connected()) {
            publishDownsampledWaveform();
            publishFullWaveformInChunks(local_requestedAxis);
        }
    } 
    else if (local_isReadingSpectrum) {
        uint16_t START_ADDRESS = 0;
        if (local_requestedAxis == AXIS_X) START_ADDRESS = ADDR_SPEC_X_START;
        else if (local_requestedAxis == AXIS_Y) START_ADDRESS = ADDR_SPEC_Y_START;
        else if (local_requestedAxis == AXIS_Z) START_ADDRESS = ADDR_SPEC_Z_START;
        else { 
            logMessage("[Spectrum] Capture failed: Invalid Axis.");
             if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                isReadingSpectrum = false;
                xSemaphoreGive(dataMutex);
            }
            return; 
        }

        bool success = readLargeDataBlock(START_ADDRESS, SPECTRUM_TOTAL_POINTS, spectrumData);
        
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            isReadingSpectrum = false; 
            String ts = getTimestamp();
            if (success) {
                newSpectrumAvailable = true;
                
                // --- UPDATE MODBUS STATUS FOR PLC ---
                if(modbusTcpEnabled) {
                    uint16_t statusVal = 0;
                    if(local_requestedAxis == AXIS_X) statusVal = 2;
                    else if(local_requestedAxis == AXIS_Y) statusVal = 3;
                    else if(local_requestedAxis == AXIS_Z) statusVal = 4;
                    modbusTCPServer.Hreg(HREG_SPECTRUM_READY_STATUS, statusVal);
                }

                strncpy(lastCaptureTimestamp, ts.c_str(), sizeof(lastCaptureTimestamp) - 1);
                lastCapturedAxis = local_requestedAxis;
                snprintf(systemStatus, sizeof(systemStatus), "Capture complete for %c-axis.", axisChar);
                strcpy(trafficLightStatus, "READY"); 
                lastCaptureActionTime = millis();
                
                if (local_isAutoCaptureCycleActive) {
                    autoCaptureAxisIndex = (autoCaptureAxisIndex + 1) % 3;
                    if (autoCaptureAxisIndex != 0) { 
                        switch(autoCaptureAxisIndex) {
                            case 1: requestedAxis = AXIS_Y; break;
                            case 2: requestedAxis = AXIS_Z; break;
                        }
                        currentDataState = DD_REQUEST_DATA; 
                    } else { 
                        strncpy(lastAutoCaptureTimestamp, ts.c_str(), sizeof(lastAutoCaptureTimestamp) - 1);
                        lastAutoCaptureMillis = millis();
                        isAutoCaptureCycleActive = false;
                        currentDataState = DD_IDLE;
                        requestedCapture = CAPTURE_NONE;
                        requestedAxis = AXIS_NONE;
                    }
                } else {
                    currentDataState = DD_IDLE;
                    requestedCapture = CAPTURE_NONE;
                    requestedAxis = AXIS_NONE;
                }
                
            } else {
                snprintf(systemStatus, sizeof(systemStatus), "Spectrum Download Failed");
                strcpy(trafficLightStatus, "ERROR");
                currentDataState = DD_IDLE;
                isAutoCaptureCycleActive = false;
            }
            xSemaphoreGive(dataMutex); 
            
            if (success) {
                 if(mqttClient && mqttClient->connected()) {
                    publishFullSpectrumInChunks(local_requestedAxis);
                    
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memcpy(globalScratchBuffer, spectrumData, SPECTRUM_TOTAL_POINTS * sizeof(int16_t));
                        xSemaphoreGive(dataMutex);
                    }
                    std::vector<SpectrumPeak> peaks = findSpectrumPeaks(globalScratchBuffer, 5);
                    
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        local_isAutoCaptureCycleActive = isAutoCaptureCycleActive;
                        xSemaphoreGive(dataMutex);
                    }
                    
                    if (local_isAutoCaptureCycleActive) {
                        xyzPeakDataCache[local_requestedAxis] = peaks;
                    } else {
                        publishSpectrumPeaks(local_requestedAxis, peaks);
                    }
                    
                    if(autoCaptureAxisIndex == 0 && xyzPeakDataCache.size() > 0) { 
                        publishCombinedSpectrumPeaks(); 
                    }
                } else {
                    logMessage("Failed to allocate for peak finding.");
                }
            }
        }
    }
}

// =================================================================
// --- MODBUS TCP HELPERS (LINEAR MAPPING)
// =================================================================
void updateModbusTCPCache() {
    if (!modbusTcpEnabled) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // X
        modbusTCPServer.Hreg(0, (int16_t)(accelRmsX * 100));
        modbusTCPServer.Hreg(1, (int16_t)(accelPeakX * 100));
        modbusTCPServer.Hreg(2, (int16_t)(accelRms2X * 100));
        modbusTCPServer.Hreg(3, (int16_t)(accelPeak2X * 100));
        modbusTCPServer.Hreg(4, (int16_t)(veloRmsX * 10));
        modbusTCPServer.Hreg(5, (int16_t)(veloPeakX * 10));
        modbusTCPServer.Hreg(6, (int16_t)(displRmsX * 1));
        modbusTCPServer.Hreg(7, (int16_t)(displPeakX * 1));
        modbusTCPServer.Hreg(8, (int16_t)(truePeakX * 100));
        modbusTCPServer.Hreg(9, (int16_t)(crestFactorX * 100));
        modbusTCPServer.Hreg(10, (int16_t)(accelMetricRmsX * 10));
        modbusTCPServer.Hreg(11, (int16_t)(accelMetricPeakX * 10));
        modbusTCPServer.Hreg(12, (int16_t)(accelMetricRms2X * 10));
        modbusTCPServer.Hreg(13, (int16_t)(accelMetricPeak2X * 10));
        modbusTCPServer.Hreg(14, (int16_t)(veloWideRmsX * 10));
        modbusTCPServer.Hreg(15, (int16_t)(veloWidePeakX * 10));
        modbusTCPServer.Hreg(16, (int16_t)(displLowRmsX * 1));
        modbusTCPServer.Hreg(17, (int16_t)(displLowPeakX * 1));
        // Y
        modbusTCPServer.Hreg(20, (int16_t)(accelRmsY * 100));
        modbusTCPServer.Hreg(21, (int16_t)(accelPeakY * 100));
        modbusTCPServer.Hreg(22, (int16_t)(accelRms2Y * 100));
        modbusTCPServer.Hreg(23, (int16_t)(accelPeak2Y * 100));
        modbusTCPServer.Hreg(24, (int16_t)(veloRmsY * 10));
        modbusTCPServer.Hreg(25, (int16_t)(veloPeakY * 10));
        modbusTCPServer.Hreg(26, (int16_t)(displRmsY * 1));
        modbusTCPServer.Hreg(27, (int16_t)(displPeakY * 1));
        modbusTCPServer.Hreg(28, (int16_t)(truePeakY * 100));
        modbusTCPServer.Hreg(29, (int16_t)(crestFactorY * 100));
        modbusTCPServer.Hreg(30, (int16_t)(accelMetricRmsY * 10));
        modbusTCPServer.Hreg(31, (int16_t)(accelMetricPeakY * 10));
        modbusTCPServer.Hreg(32, (int16_t)(accelMetricRms2Y * 10));
        modbusTCPServer.Hreg(33, (int16_t)(accelMetricPeak2Y * 10));
        modbusTCPServer.Hreg(34, (int16_t)(veloWideRmsY * 10));
        modbusTCPServer.Hreg(35, (int16_t)(veloWidePeakY * 10));
        modbusTCPServer.Hreg(36, (int16_t)(displLowRmsY * 1));
        modbusTCPServer.Hreg(37, (int16_t)(displLowPeakY * 1));
        // Z
        modbusTCPServer.Hreg(40, (int16_t)(accelRmsZ * 100));
        modbusTCPServer.Hreg(41, (int16_t)(accelPeakZ * 100));
        modbusTCPServer.Hreg(42, (int16_t)(accelRms2Z * 100));
        modbusTCPServer.Hreg(43, (int16_t)(accelPeak2Z * 100));
        modbusTCPServer.Hreg(44, (int16_t)(veloRmsZ * 10));
        modbusTCPServer.Hreg(45, (int16_t)(veloPeakZ * 10));
        modbusTCPServer.Hreg(46, (int16_t)(displRmsZ * 1));
        modbusTCPServer.Hreg(47, (int16_t)(displPeakZ * 1));
        modbusTCPServer.Hreg(48, (int16_t)(truePeakZ * 100));
        modbusTCPServer.Hreg(49, (int16_t)(crestFactorZ * 100));
        modbusTCPServer.Hreg(50, (int16_t)(accelMetricRmsZ * 10));
        modbusTCPServer.Hreg(51, (int16_t)(accelMetricPeakZ * 10));
        modbusTCPServer.Hreg(52, (int16_t)(accelMetricRms2Z * 10));
        modbusTCPServer.Hreg(53, (int16_t)(accelMetricPeak2Z * 10));
        modbusTCPServer.Hreg(54, (int16_t)(veloWideRmsZ * 10));
        modbusTCPServer.Hreg(55, (int16_t)(veloWidePeakZ * 10));
        modbusTCPServer.Hreg(56, (int16_t)(displLowRmsZ * 1));
        modbusTCPServer.Hreg(57, (int16_t)(displLowPeakZ * 1));
        // Other
        modbusTCPServer.Hreg(60, (int16_t)(temperature * 10));

        xSemaphoreGive(dataMutex);
    }
}

// --- COPY FULL BUFFERS TO MODBUS REGISTERS ---
void syncModbusRegisters() {
    if (!modbusTcpEnabled) return;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        
        if (newWaveformAvailable) {
            for (int i = 0; i < WAVEFORM_TOTAL_POINTS; i++) {
                modbusTCPServer.Hreg(HREG_WAVEFORM_START + i, fullWaveform[i]);
            }
        }

        if (newSpectrumAvailable) {
            for (int i = 0; i < SPECTRUM_TOTAL_POINTS; i++) {
                modbusTCPServer.Hreg(HREG_SPECTRUM_START + i, spectrumData[i]);
            }
        }

        xSemaphoreGive(dataMutex);
    }
}
// =================================================================
// --- OTA CONFIGURATION
// =================================================================
void setupOTA() {
  ArduinoOTA.setHostname("ILA-Gateway-883M");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) type = "sketch";
    else type = "filesystem";
    logMessage("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    logMessage("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });

  ArduinoOTA.onError([](ota_error_t error) {
    String err = "Error[" + String(error) + "]: ";
    if (error == OTA_AUTH_ERROR) err += "Auth Failed";
    else if (error == OTA_BEGIN_ERROR) err += "Begin Failed";
    else if (error == OTA_CONNECT_ERROR) err += "Connect Failed";
    else if (error == OTA_RECEIVE_ERROR) err += "Receive Failed";
    else if (error == OTA_END_ERROR) err += "End Failed";
    logMessage(err);
  });

  ArduinoOTA.begin();
  logMessage("OTA Service Started");
}

// =================================================================
// --- RTOS TASKS ---
// =================================================================
void webServerLoop(void * pvParameters) {
    for(;;) { 
        server.handleClient(); 
        ArduinoOTA.handle(); // Check for OTA updates
        vTaskDelay(2); 
    }
}

void backgroundLoop(void * pvParameters) {
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 60000,
        .idle_core_mask = (1 << 1), 
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);
    esp_task_wdt_add(NULL); 
    
    for(;;) {
        // --- FIX: Feed watchdog every loop iteration ---
        esp_task_wdt_reset(); 

        // --- NEW: Safe Reboot Handler ---
        if (shouldReboot) {
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP.restart();
        }
        // --------------------------------

        bool isWifiConnected = (WiFi.status() == WL_CONNECTED);

        if (digitalRead(USER_BUTTON_PIN) == LOW) { 
            if (buttonPressStartTime == 0) {
                buttonPressStartTime = millis();
            }
            if (millis() - buttonPressStartTime > 5000 && !apModeTriggeredByButton) {
                apModeTriggeredByButton = true;
                logMessage("AP Mode Forced by Button Press");
                WiFi.disconnect(true);
                WiFi.mode(WIFI_AP);
                WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
                tone(BUZZER_PIN, 2000, 500);
            }
        } else {
            buttonPressStartTime = 0;
            apModeTriggeredByButton = false;
        }

        if (wasWifiConnected && !isWifiConnected && !apModeTriggeredByButton) {
            wifiDisconnectAlarmActive = true;
            lastAlarmActionTime = millis();
            isAlarmBeeping = true;
            tone(BUZZER_PIN, 1200);
            snprintf(systemStatus, sizeof(systemStatus), "Wi-Fi Disconnected!");
            logMessage("[WiFi] Warning: Connection Lost!");
        } else if (!wasWifiConnected && isWifiConnected) {
            wifiDisconnectAlarmActive = false;
            noTone(BUZZER_PIN);
            logMessage("[WiFi] Reconnected. IP: " + WiFi.localIP().toString());
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // Resync time
        }
        wasWifiConnected = isWifiConnected;
        
        if (wifiDisconnectAlarmActive) {
            if (isAlarmBeeping && (millis() - lastAlarmActionTime >= 3000)) {
                isAlarmBeeping = false;
                noTone(BUZZER_PIN);
                lastAlarmActionTime = millis();
            } else if (!isAlarmBeeping && (millis() - lastAlarmActionTime >= 10000)) {
                isAlarmBeeping = true;
                tone(BUZZER_PIN, 1200);
                lastAlarmActionTime = millis();
            }
        }
        
        if (mqttEnabled && isWifiConnected && mqttClient) {
            if (!mqttClient->connected()) {
                if (millis() - lastMqttAttemptTime > mqttReconnectInterval) {
                    lastMqttAttemptTime = millis();
                    
                    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        snprintf(mqttStatus, sizeof(mqttStatus), "Connecting...");
                        xSemaphoreGive(dataMutex);
                    }
                    
                    esp_task_wdt_reset(); 

                    if (mqttClient->connect("esp32-ila-sensor", mqtt_user_str.c_str(), mqtt_pass_str.c_str())) {
                         logMessage("[MQTT] Connected to " + mqtt_server_str);
                         if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            snprintf(mqttStatus, sizeof(mqttStatus), "Connected");
                            xSemaphoreGive(dataMutex);
                         }
                        mqttClient->subscribe(mqtt_command_topic.c_str());
                    } else {
                         logMessage("[MQTT] Connect Failed rc=" + String(mqttClient->state()));
                    }
                }
            }
            if (mqttClient->connected()) { 
                mqttClient->loop(); 
                if (millis() - lastHealthPublishTime >= healthPublishInterval) {
                    lastHealthPublishTime = millis();
                    publishDeviceHealth();
                }
            }
        }

        if (modbusTcpEnabled) { modbusTCPServer.task(); }
        updateLedStatus();
        
        handleLongReads(); // Handles the actual Modbus reading state machine

        // Sync Modbus TCP registers if new data arrived
        static unsigned long lastModbusSync = 0;
        if (millis() - lastModbusSync > 1000) { 
             syncModbusRegisters();
             lastModbusSync = millis();
        }

        bool local_isReadingWaveform, local_isReadingSpectrum;
        if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            local_isReadingWaveform = isReadingWaveform;
            local_isReadingSpectrum = isReadingSpectrum;
            xSemaphoreGive(dataMutex);
        } else { continue; }

        if (!local_isReadingWaveform && !local_isReadingSpectrum) {
            if (millis() - lastMetricsReadTime >= metricsReadInterval_ms) { 
                lastMetricsReadTime = millis();
                bool read_ok = true;
                
                // Temporary buffers
                float t_ax, t_ay, t_az, t_apx, t_apy, t_apz, t_ax2, t_ay2, t_az2, t_apx2, t_apy2, t_apz2;
                float t_vx, t_vy, t_vz, t_vpx, t_vpy, t_vpz, t_dx, t_dy, t_dz, t_dpx, t_dpy, t_dpz;
                float t_tpx, t_tpy, t_tpz, t_cfx, t_cfy, t_cfz, t_sdx, t_sdy, t_sdz, t_t;
                float t_mamx, t_mamy, t_mamz, t_mapx, t_mapy, t_mapz, t_mam2x, t_mam2y, t_mam2z, t_map2x, t_map2y, t_map2z;
                float t_vwx, t_vwy, t_vwz, t_vwpx, t_vwpy, t_vwpz;
                float t_dlx, t_dly, t_dlz, t_dlpx, t_dlpy, t_dlpz;

                uint8_t result;

                // --- CRITICAL WDT FIX: Fail Fast on First Read ---
                result = node.readInputRegisters(0x0000, 3); 
                if (result == node.ku8MBSuccess) { 
                    t_ax = (int16_t)node.getResponseBuffer(0)/100.0; 
                    t_ay = (int16_t)node.getResponseBuffer(1)/100.0; 
                    t_az = (int16_t)node.getResponseBuffer(2)/100.0;
                } else {
                    read_ok = false;
                }
                
                if (read_ok) { // Only proceed if first read worked
                    vTaskDelay(5);
                    result = node.readInputRegisters(0x0004, 3); if(result == node.ku8MBSuccess) { t_apx=(int16_t)node.getResponseBuffer(0)/100.0; t_apy=(int16_t)node.getResponseBuffer(1)/100.0; t_apz=(int16_t)node.getResponseBuffer(2)/100.0;} else read_ok=false; vTaskDelay(5);
                    if(read_ok) { result = node.readInputRegisters(0x0008, 3); if(result == node.ku8MBSuccess) { t_mamx=(int16_t)node.getResponseBuffer(0)/10.0; t_mamy=(int16_t)node.getResponseBuffer(1)/10.0; t_mamz=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x000C, 3); if(result == node.ku8MBSuccess) { t_mapx=(int16_t)node.getResponseBuffer(0)/10.0; t_mapy=(int16_t)node.getResponseBuffer(1)/10.0; t_mapz=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0010, 3); if(result == node.ku8MBSuccess) { t_ax2=(int16_t)node.getResponseBuffer(0)/100.0; t_ay2=(int16_t)node.getResponseBuffer(1)/100.0; t_az2=(int16_t)node.getResponseBuffer(2)/100.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0014, 3); if(result == node.ku8MBSuccess) { t_apx2=(int16_t)node.getResponseBuffer(0)/100.0; t_apy2=(int16_t)node.getResponseBuffer(1)/100.0; t_apz2=(int16_t)node.getResponseBuffer(2)/100.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0018, 3); if(result == node.ku8MBSuccess) { t_mam2x=(int16_t)node.getResponseBuffer(0)/10.0; t_mam2y=(int16_t)node.getResponseBuffer(1)/10.0; t_mam2z=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x001C, 3); if(result == node.ku8MBSuccess) { t_map2x=(int16_t)node.getResponseBuffer(0)/10.0; t_map2y=(int16_t)node.getResponseBuffer(1)/10.0; t_map2z=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0020, 3); if(result == node.ku8MBSuccess) { t_vx=(int16_t)node.getResponseBuffer(0)/10.0; t_vy=(int16_t)node.getResponseBuffer(1)/10.0; t_vz=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0024, 3); if(result == node.ku8MBSuccess) { t_vpx=(int16_t)node.getResponseBuffer(0)/10.0; t_vpy=(int16_t)node.getResponseBuffer(1)/10.0; t_vpz=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0028, 3); if(result == node.ku8MBSuccess) { t_vwx=(int16_t)node.getResponseBuffer(0)/10.0; t_vwy=(int16_t)node.getResponseBuffer(1)/10.0; t_vwz=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x002C, 3); if(result == node.ku8MBSuccess) { t_vwpx=(int16_t)node.getResponseBuffer(0)/10.0; t_vwpy=(int16_t)node.getResponseBuffer(1)/10.0; t_vwpz=(int16_t)node.getResponseBuffer(2)/10.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0030, 3); if(result == node.ku8MBSuccess) { t_dlx=(int16_t)node.getResponseBuffer(0)/1.0; t_dly=(int16_t)node.getResponseBuffer(1)/1.0; t_dlz=(int16_t)node.getResponseBuffer(2)/1.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0034, 3); if(result == node.ku8MBSuccess) { t_dlpx=(int16_t)node.getResponseBuffer(0)/1.0; t_dlpy=(int16_t)node.getResponseBuffer(1)/1.0; t_dlpz=(int16_t)node.getResponseBuffer(2)/1.0; t_dpx=(int16_t)node.getResponseBuffer(0)/1.0; t_dpy=(int16_t)node.getResponseBuffer(1)/1.0; t_dpz=(int16_t)node.getResponseBuffer(2)/1.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0040, 3); if(result == node.ku8MBSuccess) { t_tpx=(int16_t)node.getResponseBuffer(0)/100.0; t_tpy=(int16_t)node.getResponseBuffer(1)/100.0; t_tpz=(int16_t)node.getResponseBuffer(2)/100.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0044, 3); if(result == node.ku8MBSuccess) { t_cfx=(int16_t)node.getResponseBuffer(0)/100.0; t_cfy=(int16_t)node.getResponseBuffer(1)/100.0; t_cfz=(int16_t)node.getResponseBuffer(2)/100.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { result = node.readInputRegisters(0x0048, 3); if(result == node.ku8MBSuccess) { t_sdx=(int16_t)node.getResponseBuffer(0)/100.0; t_sdy=(int16_t)node.getResponseBuffer(1)/100.0; t_sdz=(int16_t)node.getResponseBuffer(2)/100.0;} else read_ok=false; vTaskDelay(5); }
                    if(read_ok) { read_ok &= readSensorValue("Temperature", 0x0050, 10.0, t_t); }
                }

                if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (read_ok) {
                        // Standard Accel
                        accelRmsX = t_ax; accelRmsY = t_ay; accelRmsZ = t_az;
                        accelPeakX = t_apx; accelPeakY = t_apy; accelPeakZ = t_apz;
                        accelRms2X = t_ax2; accelRms2Y = t_ay2; accelRms2Z = t_az2;
                        accelPeak2X = t_apx2; accelPeak2Y = t_apy2; accelPeak2Z = t_apz2;
                        // New Metric Accel
                        accelMetricRmsX = t_mamx; accelMetricRmsY = t_mamy; accelMetricRmsZ = t_mamz;
                        accelMetricPeakX = t_mapx; accelMetricPeakY = t_mapy; accelMetricPeakZ = t_mapz;
                        accelMetricRms2X = t_mam2x; accelMetricRms2Y = t_mam2y; accelMetricRms2Z = t_mam2z;
                        accelMetricPeak2X = t_map2x; accelMetricPeak2Y = t_map2y; accelMetricPeak2Z = t_map2z;
                        // Standard Velo
                        veloRmsX = t_vx; veloRmsY = t_vy; veloRmsZ = t_vz;
                        veloPeakX = t_vpx; veloPeakY = t_vpy; veloPeakZ = t_vpz;
                        // New Wide Velo
                        veloWideRmsX = t_vwx; veloWideRmsY = t_vwy; veloWideRmsZ = t_vwz;
                        veloWidePeakX = t_vwpx; veloWidePeakY = t_vwpy; veloWidePeakZ = t_vwpz;
                        // Standard Disp
                        displRmsX = t_dx; displRmsY = t_dy; displRmsZ = t_dz;
                        displPeakX = t_dpx; displPeakY = t_dpy; displPeakZ = t_dpz;
                        // New Low Disp
                        displLowRmsX = t_dlx; displLowRmsY = t_dly; displLowRmsZ = t_dlz;
                        displLowPeakX = t_dlpx; displLowPeakY = t_dlpy; displLowPeakZ = t_dlpz;
                        // Other
                        truePeakX = t_tpx; truePeakY = t_tpy; truePeakZ = t_tpz;
                        crestFactorX = t_cfx; crestFactorY = t_cfy; crestFactorZ = t_cfz;
                        stdDeviationX = t_sdx; stdDeviationY = t_sdy; stdDeviationZ = t_sdz;
                        temperature = t_t;
                        
                        if (!wifiDisconnectAlarmActive) {
                            snprintf(systemStatus, sizeof(systemStatus), "Monitoring");
                        }
                        bool local_isMetricsRecording = isMetricsRecording;
                        xSemaphoreGive(dataMutex);
                        
                        if(mqttClient && mqttClient->connected()) { publishMetrics(); }
                        if(local_isMetricsRecording) { logMetrics(); }
                        if (modbusTcpEnabled) { updateModbusTCPCache(); syncModbusRegisters(); } // Use syncModbusRegisters
                    } else {
                        // --- FAULT HANDLING ---
                        snprintf(systemStatus, sizeof(systemStatus), "Sensor Disconnected");
                        logMessage("Error reading metrics: Sensor likely disconnected.");
                        strcpy(trafficLightStatus, "ERROR"); 
                        xSemaphoreGive(dataMutex);
                    }
                }
            }
            
            DynamicDataState local_currentDataState;
            long local_dataCaptureInterval;
            unsigned long local_lastAutoCaptureMillis;

            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                local_currentDataState = currentDataState;
                local_dataCaptureInterval = dataCaptureInterval;
                local_lastAutoCaptureMillis = lastAutoCaptureMillis;
                xSemaphoreGive(dataMutex);
            } else { continue; }

            switch (local_currentDataState) {
                case DD_IDLE:
                    if (local_dataCaptureInterval < INTERVAL_NEVER && (millis() - local_lastAutoCaptureMillis >= local_dataCaptureInterval)) {
                        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            requestedCapture = CAPTURE_BOTH;
                            isAutoCaptureCycleActive = true;
                            autoCaptureAxisIndex = 0; 
                            requestedAxis = AXIS_X;
                            currentDataState = DD_REQUEST_DATA;
                            xyzPeakDataCache.clear(); 
                            xSemaphoreGive(dataMutex);
                        }
                    }
                    break;
                
                case DD_REQUEST_DATA:
                    if (requestNewWaveform()) {
                        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            snprintf(systemStatus, sizeof(systemStatus), "Sensor Calculating...");
                            strcpy(trafficLightStatus, "TRIGGERED"); 
                            pollAttemptCounter = 0; 
                            lastPollTime = millis(); 
                            currentDataState = DD_WAITING_FOR_DATA;
                            xSemaphoreGive(dataMutex);
                        }
                    } else {
                        logMessage("[Sensor] Failed to request new data (Trigger failed).");
                        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            snprintf(systemStatus, sizeof(systemStatus), "Failed to request data");
                            strcpy(trafficLightStatus, "ERROR"); 
                            lastCaptureActionTime = millis(); 
                            currentDataState = DD_IDLE;
                            requestedCapture = CAPTURE_NONE;
                            isAutoCaptureCycleActive = false;
                            xSemaphoreGive(dataMutex);
                        }
                    }
                    break;

                case DD_WAITING_FOR_DATA:
                    if (millis() - lastPollTime >= dataPollInterval) { 
                        lastPollTime = millis();
                        
                        if (isWaveformReady()) { 
                            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                snprintf(systemStatus, sizeof(systemStatus), "Sensor Ready. Downloading...");
                                strcpy(trafficLightStatus, "READY"); 
                                isReadingWaveform = true; 
                                waveformReadIndex = 0;
                                spectrumReadIndex = 0;
                                newWaveformAvailable = false;
                                newSpectrumAvailable = false;
                                currentDataState = DD_IDLE; 
                                xSemaphoreGive(dataMutex);
                            }
                        } else if (pollAttemptCounter >= maxPollAttempts) {
                            logMessage("[Sensor] Timeout waiting for 0x0200 ready flag.");
                            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                snprintf(systemStatus, sizeof(systemStatus), "Timeout waiting for sensor");
                                strcpy(trafficLightStatus, "ERROR"); 
                                lastCaptureActionTime = millis(); 
                                currentDataState = DD_IDLE;
                                requestedCapture = CAPTURE_NONE;
                                isAutoCaptureCycleActive = false;
                                xSemaphoreGive(dataMutex);
                            }
                        } else {
                            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                                snprintf(systemStatus, sizeof(systemStatus), "Waiting for sensor... (Poll %d/%d)", pollAttemptCounter, maxPollAttempts);
                                strcpy(trafficLightStatus, "BUSY"); 
                                pollAttemptCounter++;
                                xSemaphoreGive(dataMutex);
                            }
                        }
                    }
                    break;
            }
        }
        vTaskDelay(10);
    }
}

// =================================================================
// --- FIRMWARE UPDATE HANDLERS (Manual OTA)
// =================================================================
void handleUpdateResult() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", "<h1>Update Complete.</h1><p>Device is rebooting... <a href='/'>Click here to return</a> in 20 seconds.</p>");
    delay(1000);
    ESP.restart();
}

void handleUpdateUpload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        logMessage("Firmware Update Started: " + upload.filename);
        // Disable Watchdog for long updates prevents reboots during flash
        esp_task_wdt_deinit(); 
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { 
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        // Flashing firmware to ESP
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { // true to set the size to the current progress
            logMessage("Firmware Update Success: " + String(upload.totalSize) + " bytes.");
        } else {
            Update.printError(Serial);
            logMessage("Firmware Update Failed.");
        }
    }
}

// =================================================================
// --- SETUP & MAIN LOOP
// =================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // 1. Initialize Mutex
    dataMutex = xSemaphoreCreateMutex();
    if (dataMutex == NULL) { ESP.restart(); }

    // 2. Capture Reset Reason immediately
    lastResetReason = getResetReasonString();
    
    // 3. Init Hardware
    pixels.begin();
    pixels.setBrightness(40);
    pixels.show();
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    // --- Boot/Reset Beep Sequence ---
    tone(BUZZER_PIN, 2000, 100); 
    delay(150);
    tone(BUZZER_PIN, 2500, 200); 
    delay(200);
    noTone(BUZZER_PIN);

    pinMode(USER_BUTTON_PIN, INPUT_PULLUP);

    // --- MAX485 Control Pin Setup ---
    pinMode(RS485_CTRL_PIN, OUTPUT);
    digitalWrite(RS485_CTRL_PIN, LOW); // Default to Receive Mode

    logMessage("--- System Starting ---");
    logMessage("Reset Reason: " + lastResetReason);

    // 4. Load Preferences
    preferences.begin("device-cfg", true); 
    dataCaptureInterval = preferences.getLong("wf_interval", 600000); 
    metricsReadInterval_ms = preferences.getLong("m_interval", 2000); 
    gmtOffset_sec = preferences.getLong("tz_offset", 28800); // Load Timezone
    preferences.end();
    
    preferences.begin("mqtt-cfg", true); 
    mqttEnabled = preferences.getBool("enabled", true);
    mqtt_use_tls = preferences.getBool("tls", true);
    mqtt_skip_cert_validation = preferences.getBool("skip_val", true);
    mqtt_publish_progress = preferences.getBool("progress", true);
    mqtt_server_str = preferences.getString("server", mqtt_server_str); 
    mqtt_port_val = preferences.getInt("port", mqtt_port_val); 
    mqtt_user_str = preferences.getString("user", mqtt_user_str); 
    mqtt_pass_str = preferences.getString("pass", mqtt_pass_str); 
    mqtt_topic_str = preferences.getString("topic", mqtt_topic_str); 
    preferences.end();

    preferences.begin("rs485-cfg", true); 
    modbus_slave_id_val = preferences.getUChar("slave_id", modbus_slave_id_val); 
    modbus_baud_rate_val = preferences.getLong("baud_rate", modbus_baud_rate_val); 
    preferences.end();
    
    preferences.begin("modbustcp-cfg", true);
    modbusTcpEnabled = preferences.getBool("enabled", true);
    modbusTcpPort = preferences.getUShort("port", 502);
    preferences.end();
    
    // 5. Connect WiFi
    String saved_ssid = "", saved_pass = "";
    preferences.begin("wifi-creds", true); saved_ssid = preferences.getString("ssid", ""); saved_pass = preferences.getString("password", ""); preferences.end();
    bool connected_to_wifi = false;
    
    if (saved_ssid.length() > 0) {
        logMessage("Connecting to Wi-Fi: " + saved_ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); attempts++; }
        if (WiFi.status() == WL_CONNECTED) { 
            connected_to_wifi = true; 
            logMessage("Wi-Fi Connected! IP: " + WiFi.localIP().toString());
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        } else { 
            logMessage("Wi-Fi Connect Failed. Starting AP Mode.");
            WiFi.disconnect(true); 
        }
    }
    if (!connected_to_wifi) { 
        WiFi.mode(WIFI_AP); 
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS); 
        logMessage("AP Mode Started: " + String(WIFI_AP_SSID));
    }

    wasWifiConnected = (WiFi.status() == WL_CONNECTED);

    // 5.1 Initialize OTA
    setupOTA();

    // 6. Init Modbus
    Serial1.begin(modbus_baud_rate_val, SERIAL_8N1, RXD_PIN, TXD_PIN);
    
    // --- Register MAX485 Callbacks ---
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
    
    node.begin(modbus_slave_id_val, Serial1);
    
    // --- Call Custom Identity Function ---
    delay(500); 
    fetchSensorIdentity(); 

    if (connected_to_wifi && modbusTcpEnabled) {
      modbusTCPServer.server(modbusTcpPort);
      // 1. Add Live Metrics Registers (0-100)
      modbusTCPServer.addHreg(0, 0, 100); 
      // 2. Add Full Waveform Block (1000 - 14334)
      modbusTCPServer.addHreg(HREG_WAVEFORM_START, 0, WAVEFORM_TOTAL_POINTS);
      // 3. Add Full Spectrum Block (20000 - 26145)
      modbusTCPServer.addHreg(HREG_SPECTRUM_START, 0, SPECTRUM_TOTAL_POINTS);
      logMessage("Modbus TCP: Map 1000->Waveform, 20000->Spectrum");
    }
    
    // 7. Setup Web Server
    server.on("/", HTTP_GET, handleRoot);
    
    // --- New Routes ---
    server.on("/home", HTTP_GET, handleRoot);
    server.on("/dashboard", HTTP_GET, handleRoot); 
    // --------------------------
    
    server.on("/data", HTTP_GET, handleMetrics);
    server.on("/fullwaveform", HTTP_GET, handleFullWaveform);
    server.on("/fullspectrum", HTTP_GET, handleFullSpectrum);
    server.on("/settings", HTTP_GET, handleSettings);
    server.on("/wifi_settings", HTTP_GET, handleWifiSettings);
    server.on("/savewifi", HTTP_GET, handleSaveWifi);
    server.on("/mqtt_settings", HTTP_GET, handleMqttSettings);
    server.on("/save_mqtt", HTTP_GET, handleSaveMqtt);
    server.on("/rs485_settings", HTTP_GET, handleRs485Settings);
    server.on("/save_rs485", HTTP_GET, handleSaveRs485);
    server.on("/trigger_capture", HTTP_GET, handleTriggerCapture);
    server.on("/set_intervals", HTTP_GET, handleSetIntervals);
    server.on("/get_intervals", HTTP_GET, handleGetIntervals);
    server.on("/set_metrics_interval", HTTP_GET, handleSetMetricsInterval); 
    server.on("/scanwifi", HTTP_GET, handleScanWifi);
    server.on("/get_metrics_log_data", HTTP_GET, handleGetMetricsLogData);
    server.on("/metrics_log_control", HTTP_GET, handleMetricsLogControl);
    server.on("/download_metrics_csv", HTTP_GET, handleDownloadMetricsCsv);
    server.on("/clear_metrics_log", HTTP_GET, handleClearMetricsLog);
    server.on("/clear_dynamic_data", HTTP_GET, handleClearDynamicData);
    server.on("/reboot", HTTP_GET, handleReboot);
    server.on("/device_info", HTTP_GET, handleDeviceInfo);
    server.on("/modbustcp_settings", HTTP_GET, handleModbusTcpSettings);
    server.on("/save_modbustcp", HTTP_GET, handleSaveModbusTcp);
    server.on("/mqtt_status", HTTP_GET, handleMqttStatus); 
    server.on("/time_settings", HTTP_GET, handleTimeSettings); 
    server.on("/save_time", HTTP_GET, handleSaveTimeSettings); 
    server.on("/set_manual_time", HTTP_GET, handleSetManualTime); 
    server.on("/clear_system_log", HTTP_GET, handleClearSystemLog);
    
    // --- Upload Handler for Manual OTA ---
    server.on("/update", HTTP_POST, handleUpdateResult, handleUpdateUpload);

    server.onNotFound(handleNotFound);
    
    server.begin();
    logMessage("Web Server Started");

    if(connected_to_wifi && mqttEnabled){
        if (mqtt_use_tls) {
            if (mqtt_skip_cert_validation) {
                espClientSecure.setInsecure();
            } 
            mqttClient = new PubSubClient(espClientSecure);
        } else {
            mqttClient = new PubSubClient(espClient);
        }

        if (mqttClient) {
            mqttClient->setServer(mqtt_server_str.c_str(), mqtt_port_val);
            mqttClient->setBufferSize(4096);
            mqttClient->setCallback(mqttCallback);
        } else {
             logMessage("FATAL: Failed to allocate MQTT client!");
        }
    }
    
    xTaskCreatePinnedToCore(webServerLoop, "WebServerTask", 12000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(backgroundLoop, "BackgroundTask", 16384, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }// End of Part 1. Handlers are in Part 2. 
// =================================================================
// --- WEB SERVER HANDLERS (PART 2) - DASHBOARD & SETTINGS
// =================================================================

void handleRoot() {
    // We send the dashboard in one block because it is mostly static HTML.
    // The browser fills in the data via Javascript fetching /data.
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Sensor Dashboard | )rawliteral" + sensor_model + R"rawliteral(</title>
    <meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://unpkg.com/lucide@latest"></script>
    <style>
        :root { --primary-color: #007bff; --light-gray: #f8f9fa; --medium-gray: #e9ecef; --dark-gray: #495057; --text-color: #212529; --success-green: #28a745; --inactive-gray: #adb5bd; }
        body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;margin:0;background-color:#f4f6f8; color: var(--text-color);}
        
        .header{background-color:#2c3e50;color:white;padding:10px 20px;display:flex;justify-content:space-between;align-items:center;box-shadow:0 2px 4px rgba(0,0,0,.1)}
        .title-container{display:flex;flex-direction:column;align-items:flex-start} .title{font-size:22px;font-weight:600} .subtitle{font-size:12px;color:#bdc3c7;font-weight:400}
        .header .menu a{color:white;text-decoration:none;font-size:15px;padding:8px 14px;border-radius:20px;margin-left:10px;border:1px solid #4a627a; display:inline-flex; align-items:center; gap: 6px;}
        .header .time-container { text-align: center; }
        .header .time{font-size:14px;color:#bdc3c7}
        
        .main-content{padding:20px; max-width: 1200px; margin: 0 auto;}
        .tabs{display:flex;border-bottom:2px solid var(--medium-gray)}
        .tab-button{background:0 0;border:none;padding:15px 20px;font-size:16px;cursor:pointer;color:#6c757d;border-bottom:2px solid transparent}
        .tab-button.active{color:var(--primary-color);border-bottom:2px solid var(--primary-color);font-weight:700}
        .tab-content{display:none;padding-top:20px}.tab-content.active{display:block}
        
        /* --- STATUS BOXES GRID --- */
        .status-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; margin-bottom: 20px; }
        .status-box { 
            background: white; border: 2px solid var(--medium-gray); border-radius: 10px; padding: 20px; text-align: center; 
            transition: all 0.3s ease; opacity: 0.6; box-shadow: 0 2px 4px rgba(0,0,0,0.05);
        }
        .status-box .box-icon { margin-bottom: 10px; color: var(--inactive-gray); }
        .status-box .box-label { font-size: 16px; font-weight: 700; color: var(--dark-gray); }
        .status-box .box-value { font-size: 12px; color: #666; font-family: monospace; font-weight: 500; margin-top: 5px;}
        
        .status-box.active { 
            border-color: var(--success-green); background-color: #f0fff4; opacity: 1; transform: translateY(-3px);
            box-shadow: 0 8px 16px rgba(40, 167, 69, 0.15);
        }
        .status-box.active .box-icon { color: var(--success-green); }
        .status-box.active .box-value { color: var(--success-green); font-weight: bold; background: white; padding: 2px 6px; border-radius: 4px; border: 1px solid #c3e6cb;}

        /* --- DETAILED TABLE STYLES --- */
        #metrics-table{width:100%;border-collapse:collapse;background-color:white;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.05);overflow:hidden;font-size:14px;}
        #metrics-table th, #metrics-table td{padding:10px 14px;border-bottom:1px solid var(--medium-gray);}
        #metrics-table thead th{background-color:var(--light-gray);font-weight:600;color:var(--dark-gray);text-align:left;}
        #metrics-table thead th:not(:first-child){text-align:right;}
        #metrics-table tbody td:not(:first-child){text-align:right;font-weight:600;color:var(--primary-color);font-family:monospace;font-size:15px;}
        #metrics-table tbody tr:nth-child(even){background-color:var(--light-gray);}
        #metrics-table .group-header th {background-color:#e9ecef;font-weight:bold;color:#343a40;padding:8px 14px; text-transform: uppercase; font-size: 12px;}
        
        .actions-panel{background-color:white;padding:20px;border-radius:8px;box-shadow:0 4px 8px rgba(0,0,0,.05);margin-bottom:20px}
        .actions-panel h3 {margin-top:0; display:flex; align-items:center; gap: 8px; font-size: 18px;} 
        .actions-panel p {margin-top:5px; margin-bottom: 15px; color:#6c757d; font-size: 14px;}
        .actions-panel button, .actions-panel select{padding:8px 12px;font-size:14px;border:1px solid #ccc;border-radius:5px; cursor: pointer; margin-right: 10px;}
        button.btn-special { background-color: #28a745; color: white; border-color: #28a745; }
        button:disabled { opacity: 0.6; cursor: not-allowed; }
        
        /* --- TRANSMISSION TAB STYLES --- */
        .trans-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .trans-card { background: white; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,.05); overflow: hidden; }
        .trans-header { background: #343a40; color: white; padding: 12px 15px; font-weight: 600; display: flex; align-items: center; justify-content: space-between; }
        .trans-body { padding: 20px; }
        .status-row { display: flex; justify-content: space-between; border-bottom: 1px solid #eee; padding: 8px 0; font-size: 14px; }
        .status-row:last-child { border-bottom: none; }
        .status-val { font-weight: 600; font-family: monospace; color: #333; }
        .json-preview { background: #2d3436; color: #55efc4; padding: 15px; border-radius: 6px; font-family: 'Courier New', monospace; font-size: 12px; height: 300px; overflow-y: auto; white-space: pre-wrap; margin-top: 15px; }
        .reg-table { width: 100%; border-collapse: collapse; font-size: 13px; margin-top: 10px; }
        .reg-table th { background: #f1f3f5; padding: 8px; text-align: left; border-bottom: 2px solid #ddd; }
        .reg-table td { padding: 6px 8px; border-bottom: 1px solid #eee; font-family: monospace; }
        
        #progressBarContainer{width:100%;background-color:var(--medium-gray);border-radius:8px;overflow:hidden;margin:15px 0;display:none}
        #progressBar{width:0%;height:24px;background-color:var(--primary-color);text-align:center;line-height:24px;color:white;font-weight:bold;transition:width .5s ease; white-space:nowrap;}
        .footer{text-align:center;font-size:12px;color:#888;margin-top:20px;}
        .badge { padding: 4px 8px; border-radius: 4px; color: white; font-size: 12px; font-weight: bold; }
        
        #toast-notification { visibility: hidden; min-width: 250px; background-color: #333; color: #fff; text-align: center; border-radius: 8px; padding: 16px; position: fixed; z-index: 10; left: 50%; transform: translateX(-50%); bottom: 30px; font-size: 16px; opacity: 0; transition: opacity 0.5s; }
        #toast-notification.show { visibility: visible; opacity: 1; }
    </style>
</head>
<body>
    <div class="header">
        <div class="title-container">
            <div class="title">)rawliteral" + sensor_model + R"rawliteral(</div>
            <div class="subtitle">Serial: )rawliteral" + sensor_serial + R"rawliteral(</div>
        </div>
        <div class="time-container"><div class="time" id="clock">--:--:--</div></div>
        <div class="menu"><a href="/settings"><i data-lucide="settings"></i>Settings</a></div>
    </div>
    
    <div class="main-content">
        <div class="tabs">
            <button class="tab-button active" onclick="openTab(event, 'Metrics')">Overall Metrics</button>
            <button class="tab-button" onclick="openTab(event, 'Transmission')">Control & Transmission</button>
        </div>

        <div id="Metrics" class="tab-content active">
             <div id="progressBarContainer"><div id="progressBar">0%</div></div>
             
             <div class="actions-panel" style="display:flex; justify-content:space-between; align-items:center; padding:15px;">
                <div>
                    <strong>Live Update Rate:</strong>
                    <select id="metrics_interval" onchange="setMetricsInterval()">
                        <option value="1000">1 Second</option>
                        <option value="2000">2 Seconds (Default)</option>
                        <option value="5000">5 Seconds</option>
                    </select>
                </div>
                <div>
                    <button onclick="window.location.href='/download_metrics_csv'"><i data-lucide="download"></i> Download CSV Log</button>
                </div>
             </div>

             <table id="metrics-table">
                <thead><tr><th>Measurement</th><th>X-Axis</th><th>Y-Axis</th><th>Z-Axis</th></tr></thead>
                
                <tbody class="group-header"><tr><th colspan="4">Velocity (ISO 10-1k Hz)</th></tr></tbody>
                <tbody>
                    <tr><td>Peak (mm/s)</td><td id="veloPeakX">--</td><td id="veloPeakY">--</td><td id="veloPeakZ">--</td></tr>
                    <tr><td>RMS (mm/s)</td><td id="veloRmsX">--</td><td id="veloRmsY">--</td><td id="veloRmsZ">--</td></tr>
                </tbody>

                <tbody class="group-header"><tr><th colspan="4">Velocity Wide (10-5k Hz)</th></tr></tbody>
                <tbody>
                    <tr><td>Peak (mm/s)</td><td id="veloWidePeakX">--</td><td id="veloWidePeakY">--</td><td id="veloWidePeakZ">--</td></tr>
                    <tr><td>RMS (mm/s)</td><td id="veloWideRmsX">--</td><td id="veloWideRmsY">--</td><td id="veloWideRmsZ">--</td></tr>
                </tbody>

                <tbody class="group-header"><tr><th colspan="4">Acceleration (g)</th></tr></tbody>
                <tbody>
                    <tr><td>RMS (2-1k Hz)</td><td id="accelRmsX">--</td><td id="accelRmsY">--</td><td id="accelRmsZ">--</td></tr>
                    <tr><td>Peak (2-1k Hz)</td><td id="accelPeakX">--</td><td id="accelPeakY">--</td><td id="accelPeakZ">--</td></tr>
                    <tr><td>RMS (10-5k Hz)</td><td id="accelRms2X">--</td><td id="accelRms2Y">--</td><td id="accelRms2Z">--</td></tr>
                    <tr><td>Peak (10-5k Hz)</td><td id="accelPeak2X">--</td><td id="accelPeak2Y">--</td><td id="accelPeak2Z">--</td></tr>
                </tbody>

                <tbody class="group-header"><tr><th colspan="4">Acceleration Metric (m/s&sup2;)</th></tr></tbody>
                <tbody>
                    <tr><td>RMS (2-1k Hz)</td><td id="accelMetricRmsX">--</td><td id="accelMetricRmsY">--</td><td id="accelMetricRmsZ">--</td></tr>
                    <tr><td>Peak (2-1k Hz)</td><td id="accelMetricPeakX">--</td><td id="accelMetricPeakY">--</td><td id="accelMetricPeakZ">--</td></tr>
                    <tr><td>RMS (10-5k Hz)</td><td id="accelMetricRms2X">--</td><td id="accelMetricRms2Y">--</td><td id="accelMetricRms2Z">--</td></tr>
                    <tr><td>Peak (10-5k Hz)</td><td id="accelMetricPeak2X">--</td><td id="accelMetricPeak2Y">--</td><td id="accelMetricPeak2Z">--</td></tr>
                </tbody>

                <tbody class="group-header"><tr><th colspan="4">Displacement (&micro;m)</th></tr></tbody>
                <tbody>
                    <tr><td>RMS (10-1k Hz)</td><td id="displRmsX">--</td><td id="displRmsY">--</td><td id="displRmsZ">--</td></tr>
                    <tr><td>Peak (10-1k Hz)</td><td id="displPeakX">--</td><td id="displPeakY">--</td><td id="displPeakZ">--</td></tr>
                    <tr><td>RMS (2-1k Hz)</td><td id="displLowRmsX">--</td><td id="displLowRmsY">--</td><td id="displLowRmsZ">--</td></tr>
                    <tr><td>Peak (2-1k Hz)</td><td id="displLowPeakX">--</td><td id="displLowPeakY">--</td><td id="displLowPeakZ">--</td></tr>
                </tbody>
                
                <tbody class="group-header"><tr><th colspan="4">Overall & Temperature</th></tr></tbody>
                <tbody>
                    <tr><td>True Peak (g)</td><td id="truePeakX">--</td><td id="truePeakY">--</td><td id="truePeakZ">--</td></tr>
                    <tr><td>Crest Factor</td><td id="crestFactorX">--</td><td id="crestFactorY">--</td><td id="crestFactorZ">--</td></tr>
                    <tr><td>Std Deviation</td><td id="stdDeviationX">--</td><td id="stdDeviationY">--</td><td id="stdDeviationZ">--</td></tr>
                    <tr><td>Temperature (&deg;C)</td><td colspan="3" id="temperature-cell" style="text-align:right">--</td></tr>
                </tbody>
            </table>
        </div>

        <div id="Transmission" class="tab-content">
            
            <div class="actions-panel">
                <h3><i data-lucide="hard-drive"></i> Data Buffer Status</h3>
                <div class="status-grid">
                    <div id="box-x" class="status-box">
                        <div class="box-icon"><i data-lucide="activity"></i></div>
                        <div class="box-label">X-AXIS</div>
                        <div class="box-value">EMPTY</div>
                    </div>
                    <div id="box-y" class="status-box">
                        <div class="box-icon"><i data-lucide="activity"></i></div>
                        <div class="box-label">Y-AXIS</div>
                        <div class="box-value">EMPTY</div>
                    </div>
                    <div id="box-z" class="status-box">
                        <div class="box-icon"><i data-lucide="activity"></i></div>
                        <div class="box-label">Z-AXIS</div>
                        <div class="box-value">EMPTY</div>
                    </div>
                </div>
            </div>

            <div class="actions-panel">
                <h3><i data-lucide="scan-line"></i> Control Panel</h3>
                <p>Trigger a capture to populate the buffer and send via MQTT/Modbus.</p>
                <div style="display:flex; gap:15px; flex-wrap:wrap; align-items:center; background: #f8f9fa; padding: 15px; border-radius: 8px;">
                    <select id="manual_axis_select" style="min-width: 120px;">
                        <option value="X">X-Axis</option>
                        <option value="Y">Y-Axis</option>
                        <option value="Z">Z-Axis</option>
                    </select>
                    <button id="collectSingleBtn" onclick="triggerManualCapture()"><i data-lucide="scan-search"></i>Capture Selected</button>
                    <div style="width:1px; height:30px; background:#ccc; margin:0 10px;"></div>
                    <button id="collectAllBtn" class="btn-special" onclick="triggerAll()"><i data-lucide="scan"></i>Capture All (XYZ Cycle)</button>
                </div>
                
                <div style="margin-top: 15px; border-top: 1px dashed #ccc; padding-top: 10px;">
                    <label><strong>Auto-Capture Interval (MQTT):</strong></label>
                    <select id="data_interval" onchange="setIntervals()">
                        <option value="300000">5 Minutes</option>
                        <option value="600000">10 Minutes</option>
                        <option value="1800000">30 Minutes</option>
                        <option value="3600000">1 Hour</option>
                        <option value="2147483647">Never (Manual Only)</option>
                    </select>
                </div>
            </div>

            <div class="trans-grid">
                <div class="trans-card">
                    <div class="trans-header">
                        <span><i data-lucide="cloud"></i> MQTT Protocol</span>
                        <span id="mqtt-badge" class="badge" style="background:#dc3545;">Disconnected</span>
                    </div>
                    <div class="trans-body">
                        <div class="status-row"><span>Broker Address:</span><span class="status-val" id="mqtt-broker">--</span></div>
                        <div class="status-row"><span>Target Topic:</span><span class="status-val" id="mqtt-topic">--</span></div>
                        <div class="status-row"><span>Last Status:</span><span class="status-val" id="mqtt-msg">Waiting...</span></div>
                        
                        <div style="margin-top:15px; font-size:12px; font-weight:600; color:#666;">LIVE DATA PREVIEW (JSON):</div>
                        <div class="json-preview" id="mqtt-preview">Waiting for data...</div>
                    </div>
                </div>

                <div class="trans-card">
                    <div class="trans-header">
                        <span><i data-lucide="server"></i> Modbus TCP Server</span>
                        <span id="modbus-badge" class="badge" style="background:#ccc;">Checking...</span>
                    </div>
                    <div class="trans-body">
                        <div class="status-row"><span>Server Port:</span><span class="status-val" id="modbus-port">--</span></div>
                        <div class="status-row"><span>Slave ID:</span><span class="status-val" id="modbus-id">--</span></div>
                        <div class="status-row"><span>Holding Registers:</span><span class="status-val">Available</span></div>

                        <div style="margin-top:15px; font-size:12px; font-weight:600; color:#666;">REGISTER MAP GUIDE:</div>
                        <table class="reg-table">
                            <thead><tr><th>Address</th><th>Data Content</th><th>Scale</th></tr></thead>
                            <tbody>
                                <tr><td>0 - 60</td><td>Live Overall Metrics</td><td>Mixed</td></tr>
                                <tr><td>80 - 93</td><td>Control & Status Flags</td><td>Int16</td></tr>
                                <tr><td>1000+</td><td>Waveform Data Blocks</td><td>x0.001 g</td></tr>
                                <tr><td>20000+</td><td>Spectrum Data Blocks</td><td>x0.001 g</td></tr>
                            </tbody>
                        </table>
                    </div>
                </div>
            </div>
        </div>
    </div>
    <div class="footer"><span id="status">Status: Initializing...</span></div>
    <div id="toast-notification">Notification</div>
    
    <script>
        let dashboardUpdateInterval;
        let wasCapturing = false;
        let isFullCycle = false;

        function showToast(message) {
            const toast = document.getElementById("toast-notification");
            toast.textContent = message;
            toast.className = "show";
            setTimeout(() => { toast.className = toast.className.replace("show", ""); }, 3000);
        }

        function openTab(evt, tabName) {
            let tabcontent = document.getElementsByClassName("tab-content");
            for (let i = 0; i < tabcontent.length; i++) { tabcontent[i].style.display = "none"; }
            let tablinks = document.getElementsByClassName("tab-button");
            for (let i = 0; i < tablinks.length; i++) { tablinks[i].className = tablinks[i].className.replace(" active", ""); }
            document.getElementById(tabName).style.display = "block";
            evt.currentTarget.className += " active";
        }
        
        function updateClock() { document.getElementById('clock').textContent = new Date().toLocaleTimeString(); }

        function setMetricsInterval() {
            const interval = document.getElementById('metrics_interval').value;
            fetch(`/set_metrics_interval?interval=${interval}`).then(r => {
                if(r.ok) { 
                    showToast("Live update rate set to " + interval/1000 + "s"); 
                    clearInterval(dashboardUpdateInterval);
                    dashboardUpdateInterval = setInterval(updateDashboardData, interval);
                }
            });
        }

        function setIntervals() { 
            const val = document.getElementById('data_interval').value;
            fetch(`/set_intervals?interval=${val}`).then(() => showToast('Auto-Capture Interval Updated!')); 
        }

        function trigger(axis, isCycle = false) {
            fetch('/trigger_capture?axis=' + axis + '&cycle=' + isCycle)
                .then(() => console.log('Capture triggered for ' + axis + '-Axis'));
        }
        
        function triggerManualCapture() {
            const selectedAxis = document.getElementById('manual_axis_select').value;
            isFullCycle = false;
            showToast(`Requesting ${selectedAxis}-Axis Capture...`);
            trigger(selectedAxis, false);
        }

        function triggerAll() {
            showToast('Starting XYZ Cycle...');
            isFullCycle = true;
            trigger('X', true); 
        }

        function updateStatusBoxes(capturedAxis) {
            ['x', 'y', 'z'].forEach(axis => {
                const box = document.getElementById('box-' + axis);
                if(box) {
                    box.classList.remove('active');
                    box.querySelector('.box-value').textContent = "EMPTY";
                }
            });
            if (capturedAxis && capturedAxis !== 'NONE') {
                const activeBox = document.getElementById('box-' + capturedAxis.toLowerCase());
                if (activeBox) {
                    activeBox.classList.add('active');
                    activeBox.querySelector('.box-value').textContent = "DATA IN MEMORY";
                }
            }
        }
        
        function updateDashboardData() {
            fetch("/data")
            .then(r => r.json())
            .then(data => {
                const statusText = data.status || "Monitoring";
                const isCapturing = statusText.toLowerCase().includes('collecting') || statusText.toLowerCase().includes('waiting') || statusText.toLowerCase().includes('requesting') || statusText.toLowerCase().includes('downloading');
                
                document.getElementById('status').textContent = 'Status: ' + statusText;
                
                // Progress Bar
                const progressBarContainer = document.getElementById('progressBarContainer');
                const progressBar = document.getElementById('progressBar');
                if (isCapturing) {
                    progressBarContainer.style.display = 'block';
                    const matches = statusText.match(/\(([^,]+, )?(\d+)%\)/);
                    const percentage = matches ? parseInt(matches[2], 10) : 0;
                    progressBar.style.width = percentage + '%';
                    progressBar.textContent = statusText;
                } else {
                    progressBarContainer.style.display = 'none';
                }

                updateStatusBoxes(data.capturedAxis);

                // --- UPDATE METRICS TABLE (FULL LIST) ---
                try {
                    const metrics = [
                        'veloPeak', 'veloRms', 
                        'veloWidePeak', 'veloWideRms',
                        'accelPeak', 'accelRms', 
                        'accelPeak2', 'accelRms2', 
                        'accelMetricPeak', 'accelMetricRms',
                        'accelMetricPeak2', 'accelMetricRms2',
                        'displPeak', 'displRms',
                        'displLowPeak', 'displLowRms',
                        'truePeak', 'crestFactor', 'stdDeviation'
                    ];

                    if (data.accelRms) {
                        metrics.forEach(k => {
                            if(data[k]) {
                                const elX = document.getElementById(k+'X');
                                const elY = document.getElementById(k+'Y');
                                const elZ = document.getElementById(k+'Z');
                                if(elX) elX.textContent = data[k].x.toFixed(3);
                                if(elY) elY.textContent = data[k].y.toFixed(3);
                                if(elZ) elZ.textContent = data[k].z.toFixed(3);
                            }
                        });
                        document.getElementById('temperature-cell').textContent = data.temperature.toFixed(2);
                    }
                } catch(e) { console.error("Error updating table:", e); }

                // --- UPDATE TRANSMISSION TAB ---
                // MQTT
                const mqttBadge = document.getElementById('mqtt-badge');
                if(data.mqtt_connected) {
                    mqttBadge.textContent = "CONNECTED";
                    mqttBadge.style.background = "#28a745";
                } else {
                    mqttBadge.textContent = "DISCONNECTED";
                    mqttBadge.style.background = "#dc3545";
                }
                
                document.getElementById('mqtt-broker').textContent = data.mqtt_server || "Unknown";
                document.getElementById('mqtt-topic').textContent = data.mqtt_topic || "Unknown";
                document.getElementById('mqtt-msg').textContent = data.status;
                
                if(data.accelRms) {
                    const previewData = {
                        timestamp: new Date().toISOString(),
                        model: ")rawliteral" + sensor_model + R"rawliteral(",
                        type: "metrics",
                        data: {
                            accel: data.accelRms,
                            velo: data.veloRms,
                            temp: data.temperature
                        }
                    };
                    document.getElementById('mqtt-preview').textContent = JSON.stringify(previewData, null, 2);
                }

                // Modbus
                const mbBadge = document.getElementById('modbus-badge');
                if(data.modbus_enabled) {
                    mbBadge.textContent = "RUNNING";
                    mbBadge.style.background = "#28a745";
                } else {
                    mbBadge.textContent = "DISABLED";
                    mbBadge.style.background = "#6c757d";
                }
                document.getElementById('modbus-port').textContent = data.modbus_port || "502";
                document.getElementById('modbus-id').textContent = data.modbus_id || "1";

                // Cycle Logic
                if (wasCapturing && !isCapturing && isFullCycle && data.capturedAxis) {
                   if (data.capturedAxis === 'X') { setTimeout(() => trigger('Y', true), 1000); }
                   else if (data.capturedAxis === 'Y') { setTimeout(() => trigger('Z', true), 1000); }
                   else if (data.capturedAxis === 'Z') { isFullCycle = false; showToast("XYZ Cycle Complete"); }
                }
                
                wasCapturing = isCapturing;
            })
            .catch(err => console.log("Fetch error (normal if booting):", err));
        }

        window.onload = function() {
            if (typeof lucide !== 'undefined') lucide.createIcons();
            
            // Get initial interval settings
            fetch('/get_intervals').then(r => r.json()).then(d => {
                document.getElementById('metrics_interval').value = d.metrics_interval;
                document.getElementById('data_interval').value = d.interval;
                dashboardUpdateInterval = setInterval(updateDashboardData, d.metrics_interval);
            });

            updateClock();
            setInterval(updateClock, 1000);
            updateDashboardData();
        };
    </script>
</body>
</html>)rawliteral";
    server.send(200, "text/html", html);
}

void handleMetrics() {
    DynamicJsonDocument doc(4096); 
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        
        // --- 1. Populate All Metrics (Full Table Support) ---
        JsonObject accelRms = doc.createNestedObject("accelRms");
        accelRms["x"] = accelRmsX; accelRms["y"] = accelRmsY; accelRms["z"] = accelRmsZ;
        JsonObject accelPeak = doc.createNestedObject("accelPeak");
        accelPeak["x"] = accelPeakX; accelPeak["y"] = accelPeakY; accelPeak["z"] = accelPeakZ;
        JsonObject accelRms2 = doc.createNestedObject("accelRms2");
        accelRms2["x"] = accelRms2X; accelRms2["y"] = accelRms2Y; accelRms2["z"] = accelRms2Z;
        JsonObject accelPeak2 = doc.createNestedObject("accelPeak2");
        accelPeak2["x"] = accelPeak2X; accelPeak2["y"] = accelPeak2Y; accelPeak2["z"] = accelPeak2Z;
        
        JsonObject amRms = doc.createNestedObject("accelMetricRms");
        amRms["x"] = accelMetricRmsX; amRms["y"] = accelMetricRmsY; amRms["z"] = accelMetricRmsZ;
        JsonObject amPeak = doc.createNestedObject("accelMetricPeak");
        amPeak["x"] = accelMetricPeakX; amPeak["y"] = accelMetricPeakY; amPeak["z"] = accelMetricPeakZ;
        JsonObject amRms2 = doc.createNestedObject("accelMetricRms2");
        amRms2["x"] = accelMetricRms2X; amRms2["y"] = accelMetricRms2Y; amRms2["z"] = accelMetricRms2Z;
        JsonObject amPeak2 = doc.createNestedObject("accelMetricPeak2");
        amPeak2["x"] = accelMetricPeak2X; amPeak2["y"] = accelMetricPeak2Y; amPeak2["z"] = accelMetricPeak2Z;

        JsonObject veloRms = doc.createNestedObject("veloRms");
        veloRms["x"] = veloRmsX; veloRms["y"] = veloRmsY; veloRms["z"] = veloRmsZ;
        JsonObject veloPeak = doc.createNestedObject("veloPeak");
        veloPeak["x"] = veloPeakX; veloPeak["y"] = veloPeakY; veloPeak["z"] = veloPeakZ;
        
        JsonObject vwRms = doc.createNestedObject("veloWideRms");
        vwRms["x"] = veloWideRmsX; vwRms["y"] = veloWideRmsY; vwRms["z"] = veloWideRmsZ;
        JsonObject vwPeak = doc.createNestedObject("veloWidePeak");
        vwPeak["x"] = veloWidePeakX; vwPeak["y"] = veloWidePeakY; vwPeak["z"] = veloWidePeakZ;
        
        JsonObject displRms = doc.createNestedObject("displRms");
        displRms["x"] = displRmsX; displRms["y"] = displRmsY; displRms["z"] = displRmsZ;
        JsonObject displPeak = doc.createNestedObject("displPeak");
        displPeak["x"] = displPeakX; displPeak["y"] = displPeakY; displPeak["z"] = displPeakZ;
        
        JsonObject dlRms = doc.createNestedObject("displLowRms");
        dlRms["x"] = displLowRmsX; dlRms["y"] = displLowRmsY; dlRms["z"] = displLowRmsZ;
        JsonObject dlPeak = doc.createNestedObject("displLowPeak");
        dlPeak["x"] = displLowPeakX; dlPeak["y"] = displLowPeakY; dlPeak["z"] = displLowPeakZ;

        JsonObject truePeak = doc.createNestedObject("truePeak");
        truePeak["x"] = truePeakX; truePeak["y"] = truePeakY; truePeak["z"] = truePeakZ;
        JsonObject crestFactor = doc.createNestedObject("crestFactor");
        crestFactor["x"] = crestFactorX; crestFactor["y"] = crestFactorY; crestFactor["z"] = crestFactorZ;
        JsonObject stdDeviation = doc.createNestedObject("stdDeviation");
        stdDeviation["x"] = stdDeviationX; stdDeviation["y"] = stdDeviationY; stdDeviation["z"] = stdDeviationZ;
        
        doc["temperature"] = temperature;
        doc["status"] = systemStatus;
        doc["traffic_light"] = trafficLightStatus; 
        
        // --- 2. Add Configuration Details for the Transmission Tab ---
        doc["mqtt_server"] = mqtt_server_str;
        doc["mqtt_topic"] = mqtt_topic_str;
        doc["modbus_enabled"] = modbusTcpEnabled;
        doc["modbus_port"] = modbusTcpPort;
        doc["modbus_id"] = modbus_slave_id_val;
        
        switch(lastCapturedAxis) {
            case AXIS_X: doc["capturedAxis"] = "X"; break;
            case AXIS_Y: doc["capturedAxis"] = "Y"; break;
            case AXIS_Z: doc["capturedAxis"] = "Z"; break;
            default: doc["capturedAxis"] = "NONE"; break;
        }
        
        xSemaphoreGive(dataMutex);
    } else {
        doc["status"] = "System Busy";
    }
    
    // Check MQTT connection outside Mutex to avoid deadlocks
    doc["mqtt_connected"] = (mqttClient && mqttClient->connected());
    
    String payload;
    serializeJson(doc, payload);
    server.send(200, "application/json", payload);
}

// NOTE: handleFullWaveform/Spectrum are unused without graphs but kept for API compat
void handleFullWaveform() { server.send(200, "application/json", "[]"); }
void handleFullSpectrum() { server.send(200, "application/json", "{}"); }

void handleTriggerCapture() {
    String axis_str = server.arg("axis");
    bool isCycleStart = (server.arg("cycle") == "true"); 

    bool isBusy = false;
    Axis selectedAxis = AXIS_NONE;
    if (axis_str == "X") selectedAxis = AXIS_X;
    else if (axis_str == "Y") selectedAxis = AXIS_Y;
    else if (axis_str == "Z") selectedAxis = AXIS_Z;
    
    if (selectedAxis == AXIS_NONE) {
        server.send(400, "text/plain", "Invalid or missing axis parameter.");
        return;
    }

    if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        isBusy = (currentDataState != DD_IDLE);
        if (!isBusy) {
            requestedAxis = selectedAxis;
            requestedCapture = CAPTURE_BOTH; // Still captures buffer for MQTT/Modbus
            currentDataState = DD_REQUEST_DATA;

            isAutoCaptureCycleActive = isCycleStart;
            if (isCycleStart) {
                xyzPeakDataCache.clear();
                autoCaptureAxisIndex = 0;
            }
        }
        xSemaphoreGive(dataMutex);
    }
    server.send(200, "text/plain", isBusy ? "Device is busy." : "Capture trigger accepted.");
}

// --- SAFE CHUNKED SETTINGS PAGE ---
void handleSettings() {
    // 1. Tell the browser we are sending the page in parts (Unknown Length)
    server.setContentLength(CONTENT_LENGTH_UNKNOWN); 
    server.send(200, "text/html", ""); // Start the connection

    // 2. Send Part 1: The Header and CSS (Static)
    // We break this up so it doesn't use much RAM
    server.sendContent(R"rawliteral(
<!DOCTYPE html><html><head><title>Device Settings</title><meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://unpkg.com/lucide@latest"></script>
<style>body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;margin:0;background-color:#f4f6f8;display:flex;justify-content:center;align-items:center;min-height:100vh;padding:20px;box-sizing:border-box}.container{width:100%;max-width:500px;background-color:white;border-radius:12px;box-shadow:0 8px 30px rgba(0,0,0,.08);overflow:hidden}.header{text-align:center;padding:20px;border-bottom:1px solid #e9ecef}.header h2{margin:0;font-size:24px;font-weight:600;color:#2c3e50}.settings-list{list-style:none;padding:0;margin:0}.settings-item{display:flex;align-items:center;padding:15px 20px;border-bottom:1px solid #e9ecef;text-decoration:none;color:inherit;transition:background-color .2s}.settings-item:hover{background-color:#f8f9fa}.settings-item:last-child{border-bottom:none}.icon{width:24px;height:24px;margin-right:20px;color:#007bff}.content{flex-grow:1;text-align:left}.content h3{margin:0;font-size:16px;font-weight:500;color:#343a40}.content p{margin:2px 0 0;font-size:14px;color:#6c757d;display:flex;align-items:center}.chevron{width:20px;height:20px;color:#ced4da}.status-dot{height:9px;width:9px;border-radius:50%;display:inline-block;margin-right:6px}.status-dot.connected{background-color:#2ecc71}.status-dot.disconnected{background-color:#e74c3c}.actions{padding:20px;background-color:#f8f9fa;border-top:1px solid #e9ecef}.btn{display:block;width:100%;padding:12px;border:none;border-radius:8px;cursor:pointer;font-size:15px;font-weight:600;text-align:center;text-decoration:none;box-sizing:border-box;transition:background-color .2s,color .2s}.btn-danger{background-color:#e74c3c;color:white;margin-bottom:10px}.btn-danger:hover{background-color:#c0392b}.btn-secondary{background-color:transparent;color:#007bff}.btn-secondary:hover{background-color:#e9ecef}</style></head><body><div class="container"><div class="header"><h2>Device Settings</h2></div><div class="settings-list">
)rawliteral");

    // 3. Send Part 2: Wi-Fi Status (Dynamic)
    String wifiClass = (WiFi.status() == WL_CONNECTED) ? "connected" : "disconnected";
    String wifiText = (WiFi.status() == WL_CONNECTED) ? "Connected to " + WiFi.SSID() : "AP Mode";
    
    String chunk1 = R"rawliteral(<a href="/wifi_settings" class="settings-item"><div class="icon"><i data-lucide="wifi"></i></div><div class="content"><h3>Wi-Fi</h3><p><span class="status-dot )rawliteral";
    chunk1 += wifiClass + R"rawliteral("></span>)rawliteral" + wifiText + R"rawliteral(</p></div><div class="chevron"><i data-lucide="chevron-right"></i></div></a>)rawliteral";
    server.sendContent(chunk1);

    // 4. Send Part 3: MQTT Status (Dynamic)
    String mqttClass = (mqttClient && mqttClient->connected()) ? "connected" : "disconnected";
    String mqttText = (mqttEnabled) ? ((mqttClient && mqttClient->connected()) ? "Connected" : "Disconnected") : "Disabled";
    
    String chunk2 = R"rawliteral(<a href="/mqtt_settings" class="settings-item"><div class="icon"><i data-lucide="cloud"></i></div><div class="content"><h3>MQTT</h3><p><span class="status-dot )rawliteral";
    chunk2 += mqttClass + R"rawliteral("></span>)rawliteral" + mqttText + R"rawliteral(</p></div><div class="chevron"><i data-lucide="chevron-right"></i></div></a>)rawliteral";
    server.sendContent(chunk2);

    // 5. Send Part 4: Modbus Status (Dynamic)
    String mbClass = (modbusTcpEnabled) ? "connected" : "disconnected";
    String mbText = (modbusTcpEnabled) ? "Enabled" : "Disabled";

    String chunk3 = R"rawliteral(<a href="/rs485_settings" class="settings-item"><div class="icon"><i data-lucide="activity"></i></div><div class="content"><h3>Modbus RTU (RS485) Sensor</h3><p><span class="status-dot connected"></span>Ready</p></div><div class="chevron"><i data-lucide="chevron-right"></i></div></a>
<a href="/modbustcp_settings" class="settings-item"><div class="icon"><i data-lucide="server"></i></div><div class="content"><h3>Modbus TCP Server</h3><p><span class="status-dot )rawliteral";
    chunk3 += mbClass + R"rawliteral("></span>)rawliteral" + mbText + R"rawliteral(</p></div><div class="chevron"><i data-lucide="chevron-right"></i></div></a>)rawliteral";
    server.sendContent(chunk3);

    // 6. Send Part 5: The Footer and Scripts (Static)
    server.sendContent(R"rawliteral(
<a href="/time_settings" class="settings-item"><div class="icon"><i data-lucide="clock"></i></div><div class="content"><h3>Time & Date</h3><p>Timezone & Manual Set</p></div><div class="chevron"><i data-lucide="chevron-right"></i></div></a>
<a href="/device_info" class="settings-item"><div class="icon" style="color:#6c757d"><i data-lucide="info"></i></div><div class="content"><h3>Device Info</h3><p>View model, serial, and firmware</p></div><div class="chevron"><i data-lucide="chevron-right"></i></div></a>
</div><div class="actions"><button class="btn btn-danger" onclick="rebootDevice()">Reboot Device</button><a href="/" class="btn btn-secondary">Back to Dashboard</a></div></div>
<script>
    if (typeof lucide !== 'undefined') { lucide.createIcons(); }
    function rebootDevice(){if(confirm("Are you sure you want to reboot the device?")){fetch("/reboot").then(()=>{alert("Device is rebooting...")})}}
</script></body></html>)rawliteral");

    // 7. Finish the transmission
    server.sendContent(""); 
}

// --- SAFE CHUNKED WIFI SETTINGS ---
void handleWifiSettings() {
    // 1. Start the Chunked Response
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    // 2. Send Header & CSS
    server.sendContent(R"rawliteral(
<!DOCTYPE html><html><head><title>Wi-Fi Settings</title><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;margin:0;background-color:#f4f6f8;display:flex;justify-content:center;align-items:center;min-height:100vh}.container{width:100%;max-width:500px;padding:20px;box-sizing:border-box;background-color:white;border-radius:12px;box-shadow:0 8px 20px rgba(0,0,0,.1)}.header{text-align:center;padding-bottom:20px;border-bottom:2px solid #e9ecef;margin-bottom:20px}.header h2{margin:0;font-size:24px;font-weight:600;color:#2c3e50}.form-group{margin-bottom:15px}label{display:block;margin-bottom:8px;font-size:14px;font-weight:600}input[type=text],input[type=password]{width:100%;padding:12px;box-sizing:border-box;border:1px solid #ced4da;border-radius:8px;font-size:16px;transition:border-color .3s}input[type=text]:focus,input[type=password]:focus{outline:0;border-color:#007bff}.btn-primary{width:100%;padding:12px;background-color:#007bff;color:white;border:none;border-radius:8px;cursor:pointer;font-size:16px;font-weight:600;transition:background-color .3s}.btn-primary:hover{background-color:#0056b3}.btn-scan{background-color:#6c757d;margin-bottom:15px}.btn-scan:hover{background-color:#5a6268}.wifi-list-container{max-height:250px;overflow-y:auto;border:1px solid #e9ecef;border-radius:8px;padding:5px;background-color:#f8f9fa}#wifi-list{list-style:none;padding:0;margin:0}#wifi-list li{padding:12px 15px;cursor:pointer;border-bottom:1px solid #e9ecef;transition:background-color .2s;display:flex;justify-content:space-between;align-items:center}#wifi-list li:last-child{border-bottom:none}#wifi-list li:hover{background-color:#e9ecef}.wifi-ssid{font-size:16px;font-weight:500;flex-grow:1}.wifi-rssi{font-size:14px;color:#6c757d;font-weight:400}.status-bar{text-align:center;margin-top:20px;padding-top:20px;border-top:2px solid #e9ecef;font-size:14px;color:#6c757d;font-weight:500}.nav-link{display:block;text-align:center;margin-top:20px;color:#007bff;text-decoration:none;font-weight:600;font-size:14px}.nav-link:hover{text-decoration:underline}</style></head><body><div class="container"><div class="header"><h2>Wi-Fi Settings</h2></div>
)rawliteral");

    // 3. Send Dynamic Status Bar
    String statusMsg = (WiFi.status() == WL_CONNECTED ? "Connected to " + WiFi.SSID() : "AP Mode active");
    String ipMsg = (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString());
    
    server.sendContent(R"rawliteral(<div class="status-bar">Current Status: )rawliteral" + statusMsg + R"rawliteral(<br>IP Address: )rawliteral" + ipMsg + R"rawliteral(</div>)rawliteral");

    // 4. Send Form & Scripts
    server.sendContent(R"rawliteral(
<div class="form-section"><button class="btn-primary btn-scan" onclick="scanWifi()">Scan for Networks</button><div class="wifi-list-container"><ul id="wifi-list"></ul></div><form onsubmit="return saveWifi()"><div class="form-group"><label for="ssid">Network Name (SSID)</label><input type="text" id="ssid" name="ssid" required></div><div class="form-group"><label for="pass">Password</label><input type="password" id="pass" name="pass"></div><button type="submit" class="btn-primary">Save and Connect</button></form></div><a href="/settings" class="nav-link">&larr; Back to Settings</a></div><script>function scanWifi(){const t=document.getElementById("wifi-list");t.innerHTML="<li>Scanning...</li>",fetch("/scanwifi").then(t=>t.json()).then(e=>{t.innerHTML="",0===e.length?t.innerHTML="<li>No networks found.</li>":e.forEach(e=>{const n=document.createElement("li");n.innerHTML=`<span class="wifi-ssid">${e.ssid}</span><span class="wifi-rssi">${e.rssi} dBm</span>`,n.onclick=()=>{document.getElementById("ssid").value=e.ssid},t.appendChild(n)})})}function saveWifi(){const t=document.getElementById("ssid").value,e=document.getElementById("pass").value;return fetch("/savewifi?ssid="+encodeURIComponent(t)+"&pass="+encodeURIComponent(e)),alert("Saving credentials... The device will reboot."),!1}</script></body></html>
)rawliteral");

    // 5. Close Stream
    server.sendContent("");
}

void handleSaveWifi() { 
    preferences.begin("wifi-creds", false); 
    preferences.putString("ssid", server.arg("ssid")); 
    preferences.putString("password", server.arg("pass")); 
    preferences.end(); 
    server.send(200, "text/plain", "Credentials saved. Device will reboot."); 
    delay(1000); 
    ESP.restart(); 
}

// --- SAFE CHUNKED MQTT SETTINGS ---
void handleMqttSettings() {
    // 1. Start Stream
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    // 2. Header & Styles
    server.sendContent(R"rawliteral(
<!DOCTYPE html><html><head><title>MQTT Settings</title><meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://unpkg.com/lucide@latest"></script>
<style>
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;margin:0;background-color:#f4f6f8;display:flex;justify-content:center;padding:20px 0;}
.container{width:100%;max-width:500px;padding:20px;box-sizing:border-box;background-color:white;border-radius:12px;box-shadow:0 8px 20px rgba(0,0,0,.1);}
.header{text-align:center;padding-bottom:20px;border-bottom:2px solid #e9ecef;margin-bottom:20px}.header h2{margin:0;font-size:24px;font-weight:600;color:#2c3e50}
.form-group{margin-bottom:15px}.form-group.inline{display:flex;align-items:center;justify-content:space-between;padding:10px 0; border-bottom: 1px solid #f0f0f0;}
label{display:block;margin-bottom:8px;font-size:14px;font-weight:600}
input[type=text],input[type=password],input[type=number]{width:100%;padding:12px;box-sizing:border-box;border:1px solid #ced4da;border-radius:8px;font-size:16px}
.btn-primary{width:100%;padding:12px;background-color:#007bff;color:white;border:none;border-radius:8px;cursor:pointer;font-size:16px;font-weight:600}
.nav-link{display:block;text-align:center;margin-top:20px;color:#007bff;text-decoration:none}
.switch{position:relative;display:inline-block;width:50px;height:28px}.switch input{opacity:0;width:0;height:0}
.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#ccc;transition:.4s;border-radius:28px}
.slider:before{position:absolute;content:"";height:20px;width:20px;left:4px;bottom:4px;background-color:white;transition:.4s;border-radius:50%}
input:checked+.slider{background-color:#28a745} input:checked+.slider:before{transform:translateX(22px)}
#skip_validation_group { display: none; }
.status-bar{text-align:center;margin-top:20px;padding:15px;border-radius:8px;font-size:14px;font-weight:500;background-color:#f8f9fa;color:#6c757d;}
.info-box { background-color: #fff3cd; color: #856404; padding: 15px; border-radius: 8px; border: 1px solid #ffeeba; margin-bottom: 20px; font-size: 14px; line-height: 1.5; }
.info-box i { vertical-align: text-bottom; margin-right: 5px; }
.serial-badge { font-family: monospace; font-weight: bold; background: #fff; padding: 2px 6px; border-radius: 4px; border: 1px solid #e2e3e5; margin-left: 5px; color: #333; font-size: 1.1em;}
</style>
</head><body><div class="container"><div class="header"><h2>MQTT Settings</h2></div>
)rawliteral");

    // 3. Dynamic Info Box (Serial Number)
    server.sendContent(R"rawliteral(
<div class="info-box">
    <strong><i data-lucide="alert-triangle"></i> Security Requirement:</strong><br>
    All MQTT commands sent to this device <u>MUST</u> include the correct <b>serial_number</b> field.<br>
    <strong>Target Device Serial Number:</strong> <span class="serial-badge">)rawliteral" + sensor_serial + R"rawliteral(</span>
</div>
<form onsubmit="return saveMqtt()">
)rawliteral");

    // 4. Form Controls (Toggles)
    // We send these as static HTML; the Javascript at the bottom will check/uncheck them
    server.sendContent(R"rawliteral(
<div class="form-group inline"><label for="enabled" style="margin-bottom:0;">Enable MQTT Client</label><label class="switch"><input type="checkbox" id="enabled" name="enabled"><span class="slider"></span></label></div>
<div class="form-group inline"><label for="tls" style="margin-bottom:0;">Use TLS/SSL Security</label><label class="switch"><input type="checkbox" id="tls" name="tls"><span class="slider"></span></label></div>
<div class="form-group inline" id="skip_validation_group"><label for="skip_val" style="margin-bottom:0;">Skip Certificate Validation</label><label class="switch"><input type="checkbox" id="skip_val" name="skip_val"><span class="slider"></span></label></div>
<div class="form-group inline"><label for="progress" style="margin-bottom:0;">Publish Progress Updates</label><label class="switch"><input type="checkbox" id="progress" name="progress"><span class="slider"></span></label></div>
)rawliteral");

    // 5. Form Inputs (Pre-filled with Variables)
    // We break this into smaller chunks to allow variable insertion
    server.sendContent(R"rawliteral(<div class="form-group"><label for="server">Broker Address</label><input type="text" id="server" name="server" value=")rawliteral" + mqtt_server_str + R"rawliteral(" required></div>)rawliteral");
    
    server.sendContent(R"rawliteral(<div class="form-group"><label for="port">Port</label><input type="number" id="port" name="port" value=")rawliteral" + String(mqtt_port_val) + R"rawliteral(" required></div>)rawliteral");
    
    server.sendContent(R"rawliteral(<div class="form-group"><label for="user">Username</label><input type="text" id="user" name="user" value=")rawliteral" + mqtt_user_str + R"rawliteral("></div>)rawliteral");
    
    server.sendContent(R"rawliteral(<div class="form-group"><label for="pass">Password</label><input type="password" id="pass" name="pass" value=")rawliteral" + mqtt_pass_str + R"rawliteral("></div>)rawliteral");
    
    server.sendContent(R"rawliteral(<div class="form-group"><label for="topic">Base Topic</label><input type="text" id="topic" name="topic" value=")rawliteral" + mqtt_topic_str + R"rawliteral(" required></div>)rawliteral");

    // 6. Footer & Scripts (Dynamic JS Injection for Checkboxes)
    server.sendContent(R"rawliteral(
<button type="submit" class="btn-primary">Save and Reboot</button></form>
<div class="status-bar" id="mqtt-status">Loading status...</div>
<a href="/settings" class="nav-link">&larr; Back to Settings</a></div>
<script>
if (typeof lucide !== 'undefined') { lucide.createIcons(); }
function saveMqtt(){
    const enabled = document.getElementById('enabled').checked;
    const tls = document.getElementById('tls').checked;
    const skip_val = document.getElementById('skip_val').checked;
    const progress = document.getElementById('progress').checked;
    const server = document.getElementById('server').value;
    const port = document.getElementById('port').value;
    const user = document.getElementById('user').value;
    const pass = document.getElementById('pass').value;
    const topic = document.getElementById('topic').value;
    alert('Saving MQTT settings... The device will reboot.');
    const url = `/save_mqtt?enabled=${enabled}&tls=${tls}&skip_val=${skip_val}&progress=${progress}&server=${encodeURIComponent(server)}&port=${port}&user=${encodeURIComponent(user)}&pass=${encodeURIComponent(pass)}&topic=${encodeURIComponent(topic)}`;
    fetch(url).then(() => { window.location.href = '/settings'; });
    return false;
}
function updateTlsOptions() {
    const tlsEnabled = document.getElementById('tls').checked;
    document.getElementById('skip_validation_group').style.display = tlsEnabled ? 'flex' : 'none';
    const portInput = document.getElementById('port');
    if (tlsEnabled && (portInput.value == '1883' || portInput.value == '')) {
        portInput.value = '8883';
    } else if (!tlsEnabled && (portInput.value == '8883' || portInput.value == '')) {
        portInput.value = '1883';
    }
}
function updateMqttStatus() {
    fetch('/mqtt_status').then(r => r.json()).then(data => { document.getElementById('mqtt-status').textContent = data.status; });
}
window.onload = function() {
    document.getElementById('enabled').checked = )rawliteral" + String(mqttEnabled ? "true" : "false") + R"rawliteral(;
    document.getElementById('tls').checked = )rawliteral" + String(mqtt_use_tls ? "true" : "false") + R"rawliteral(;
    document.getElementById('skip_val').checked = )rawliteral" + String(mqtt_skip_cert_validation ? "true" : "false") + R"rawliteral(;
    document.getElementById('progress').checked = )rawliteral" + String(mqtt_publish_progress ? "true" : "false") + R"rawliteral(;
    document.getElementById('tls').addEventListener('change', updateTlsOptions);
    updateTlsOptions();
    updateMqttStatus();
    setInterval(updateMqttStatus, 2000);
}
</script></body></html>
)rawliteral");

    // 7. Close Stream
    server.sendContent("");
}

void handleSaveMqtt() {
    preferences.begin("mqtt-cfg", false);
    preferences.putBool("enabled", (server.arg("enabled") == "true"));
    preferences.putBool("tls", (server.arg("tls") == "true"));
    preferences.putBool("skip_val", (server.arg("skip_val") == "true"));
    preferences.putBool("progress", (server.arg("progress") == "true")); 
    preferences.putString("server", server.arg("server"));
    preferences.putInt("port", server.arg("port").toInt());
    preferences.putString("user", server.arg("user"));
    preferences.putString("pass", server.arg("pass"));
    preferences.putString("topic", server.arg("topic"));
    preferences.end();
    server.send(200, "text/plain", "MQTT settings saved. Device will reboot.");
    delay(1000);
    ESP.restart();
}

void handleRs485Settings() {
    String html = R"rawliteral(
<!DOCTYPE html><html><head><title>Modbus RS485 Settings</title><meta name="viewport" content="width=device-width, initial-scale=1"><style>body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;margin:0;background-color:#f4f6f8;display:flex;justify-content:center;align-items:center;min-height:100vh}.container{width:100%;max-width:500px;padding:20px;box-sizing:border-box;background-color:white;border-radius:12px;box-shadow:0 8px 20px rgba(0,0,0,.1)}.header{text-align:center;padding-bottom:20px;border-bottom:2px solid #e9ecef;margin-bottom:20px}.header h2{margin:0;font-size:24px;font-weight:600;color:#2c3e50}.form-group{margin-bottom:15px}label{display:block;margin-bottom:8px;font-size:14px;font-weight:600}input[type=number],select{width:100%;padding:12px;box-sizing:border-box;border:1px solid #ced4da;border-radius:8px;font-size:16px;transition:border-color .3s}input[type=number]:focus,select:focus{outline:0;border-color:#007bff}.btn-primary{width:100%;padding:12px;background-color:#007bff;color:white;border:none;border-radius:8px;cursor:pointer;font-size:16px;font-weight:600;transition:background-color .3s}.btn-primary:hover{background-color:#0056b3}.status-bar{text-align:center;margin-top:20px;padding-top:20px;border-top:2px solid #e9ecef;font-size:14px;color:#6c757d;font-weight:500}.nav-link{display:block;text-align:center;margin-top:20px;color:#007bff;text-decoration:none;font-weight:600;font-size:14px}.nav-link:hover{text-decoration:underline}</style></head><body><div class="container"><div class="header"><h2>Modbus RS485 Settings</h2></div><form onsubmit="return saveRs485()"><div class="form-group"><label for="slave_id">Slave ID</label><input type="number" id="slave_id" name="slave_id" min="1" max="247" value=")rawliteral" + String(modbus_slave_id_val) + R"rawliteral(" required></div><div class="form-group"><label for="baud_rate">Baud Rate</label><select id="baud_rate" name="baud_rate" required><option value="9600">9600</option><option value="19200">19200</option><option value="38400">38400</option><option value="57600">57600</option><option value="115200">115200</option></select></div><button type="submit" class="btn-primary">Save and Reboot</button></form><div class="status-bar" id="status-bar">Current settings are displayed above. Saving will reboot the device.</div><a href="/settings" class="nav-link">&larr; Back to Settings</a></div><script>
function saveRs485(){
    const slaveId=document.getElementById("slave_id").value;
    const baudRate=document.getElementById("baud_rate").value;
    const statusBar=document.getElementById("status-bar");
    statusBar.innerHTML="Saving RS485 settings... The device will reboot to apply changes.";
    fetch(`/save_rs485?slave_id=${slaveId}&baud_rate=${baudRate}`);
    return false;
}
window.onload=function(){document.getElementById("baud_rate").value=")rawliteral" + String(modbus_baud_rate_val) + R"rawliteral("};</script></body></html>)rawliteral";
    server.send(200, "text/html", html);
}

void handleSaveRs485() {
    preferences.begin("rs485-cfg", false);
    preferences.putUChar("slave_id", server.arg("slave_id").toInt());
    preferences.putLong("baud_rate", server.arg("baud_rate").toInt()); 
    preferences.end();
    server.send(200, "text/plain", "RS485 settings saved. Device will reboot.");
    delay(1000);
    ESP.restart();
}

void handleSetIntervals() {
    long interval = server.arg("interval").toInt();
    if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        dataCaptureInterval = interval;
        xSemaphoreGive(dataMutex);
    }
    preferences.begin("device-cfg", false);
    preferences.putLong("wf_interval", interval);
    preferences.end();
    server.send(200, "text/plain", "Interval set.");
}

void handleGetIntervals() {
    long interval = 0, metrics_interval = 0;
    if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        interval = dataCaptureInterval;
        metrics_interval = metricsReadInterval_ms;
        xSemaphoreGive(dataMutex);
    }
    server.send(200, "application/json", "{\"interval\":" + String(interval) + ", \"metrics_interval\":" + String(metrics_interval) + "}");
}

void handleScanWifi() {
    String json = "[";
    int n = WiFi.scanNetworks();
    if (n > 0) {
        for (int i = 0; i < n; ++i) {
            json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + WiFi.RSSI(i) + "}";
            if (i < n - 1) json += ",";
        }
    }
    json += "]";
    server.send(200, "application/json", json);
}

void handleGetMetricsLogData() {
    DynamicJsonDocument doc(4096); 
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        doc["is_recording"] = isMetricsRecording;
        doc["log_count"] = metricsLogCount;
        doc["max_log_size"] = METRICS_LOG_SIZE;
        JsonArray log = doc.createNestedArray("log");
        for (int i = 0; i < metricsLogCount; ++i) {
            if (metricsLog[i].is_valid) {
                JsonObject entry = log.createNestedObject();
                entry["timestamp"] = metricsLog[i].timestamp;
                JsonArray values = entry.createNestedArray("values");
                for(int j=0; j < METRICS_SNAPSHOT_SIZE; ++j) {
                    values.add(metricsLog[i].values[j]);
                }
            }
        }
        xSemaphoreGive(dataMutex);
    }
    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}

void handleMetricsLogControl() {
    String action = server.arg("action");
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (action == "start") {
            isMetricsRecording = true;
            metricsLogIndex = 0;
            metricsLogCount = 0;
            for(int i=0; i<METRICS_LOG_SIZE; ++i) metricsLog[i].is_valid = false;
        } else if (action == "stop") {
            isMetricsRecording = false;
        }
        xSemaphoreGive(dataMutex);
    }
    server.send(200, "text/plain", "OK");
}

void handleDownloadMetricsCsv() {
    String csv_header = "Timestamp,AccelRmsX,AccelRmsY,AccelRmsZ,AccelPeakX,AccelPeakY,AccelPeakZ,VeloRmsX,VeloRmsY,VeloRmsZ,VeloPeakX,VeloPeakY,VeloPeakZ,DisplRmsX,DisplRmsY,DisplRmsZ,DisplPeakX,DisplPeakY,DisplPeakZ,TruePeakX,TruePeakY,TruePeakZ,CrestFactorX,CrestFactorY,CrestFactorZ,StdDevX,StdDevY,StdDevZ,AccelRms2X,AccelRms2Y,AccelRms2Z,Temperature,AccelMetricRmsX,AccelMetricRmsY,AccelMetricRmsZ,AccelMetricPeakX,AccelMetricPeakY,AccelMetricPeakZ,AccelMetricRms2X,AccelMetricRms2Y,AccelMetricRms2Z,AccelMetricPeak2X,AccelMetricPeak2Y,AccelMetricPeak2Z,VeloWideRmsX,VeloWideRmsY,VeloWideRmsZ,VeloWidePeakX,VeloWidePeakY,VeloWidePeakZ,DisplLowRmsX,DisplLowRmsY,DisplLowRmsZ,DisplLowPeakX,DisplLowPeakY,DisplLowPeakZ\n";
    String csv_content = "";
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        for (int i = 0; i < metricsLogCount; i++) {
            if (metricsLog[i].is_valid) {
                csv_content += String(metricsLog[i].timestamp);
                for (int j = 0; j < METRICS_SNAPSHOT_SIZE; j++) {
                    csv_content += "," + String(metricsLog[i].values[j]);
                }
                csv_content += "\n";
            }
        }
        xSemaphoreGive(dataMutex);
    }
    server.sendHeader("Content-Disposition", "attachment; filename=metrics_log.csv");
    server.send(200, "text/csv", csv_header + csv_content);
}

void handleClearMetricsLog() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        metricsLogIndex = 0;
        metricsLogCount = 0;
        for(int i=0; i<METRICS_LOG_SIZE; ++i) metricsLog[i].is_valid = false;
        xSemaphoreGive(dataMutex);
    }
    server.send(200, "text/plain", "Metrics log cleared.");
}

void handleClearDynamicData() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        newWaveformAvailable = false;
        newSpectrumAvailable = false;
        
        if(modbusTcpEnabled) {
            modbusTCPServer.Hreg(HREG_WAVEFORM_READY_STATUS, 0);
            modbusTCPServer.Hreg(HREG_SPECTRUM_READY_STATUS, 0);
        }

        xSemaphoreGive(dataMutex);
    }
    server.send(200, "text/plain", "Cached data cleared.");
}

void handleReboot() {
    server.send(200, "text/plain", "Rebooting...");
    delay(1000);
    ESP.restart();
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

void handleMqttStatus() {
    DynamicJsonDocument doc(256);
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        doc["status"] = mqttStatus;
        xSemaphoreGive(dataMutex);
    } else {
        doc["status"] = "Status unavailable (mutex locked)";
    }
    String payload;
    serializeJson(doc, payload);
    server.send(200, "application/json", payload);
}

void handleSetMetricsInterval() {
    long interval = server.arg("interval").toInt();
    if (interval >= 1000) {
        if(xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            metricsReadInterval_ms = interval;
            xSemaphoreGive(dataMutex);
        }
        preferences.begin("device-cfg", false);
        preferences.putLong("m_interval", interval);
        preferences.end();
        server.send(200, "text/plain", "Metrics interval updated.");
    } else {
        server.send(400, "text/plain", "Interval too short.");
    }
}

// =================================================================
// --- NEW: Time & Date Settings Handlers ---
// =================================================================

void handleTimeSettings() {
    int offsetHours = gmtOffset_sec / 3600;
    String html = R"rawliteral(
<!DOCTYPE html><html><head><title>Time Settings</title><meta name="viewport" content="width=device-width, initial-scale=1">
<style>body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;margin:0;background-color:#f4f6f8;display:flex;justify-content:center;padding:20px 0;}
.container{width:100%;max-width:500px;background-color:white;border-radius:12px;padding:30px;box-shadow:0 8px 20px rgba(0,0,0,.1);}
h2{margin-top:0; color:#2c3e50; border-bottom:1px solid #eee; padding-bottom:15px;}
.group{margin-bottom:25px; padding:15px; background:#f8f9fa; border-radius:8px;}
label{display:block;margin-bottom:8px;font-weight:600;color:#555;}
input,select{width:100%;padding:10px;border:1px solid #ccc;border-radius:6px;box-sizing:border-box;margin-bottom:10px;}
button{width:100%;padding:12px;background:#007bff;color:white;border:none;border-radius:6px;font-weight:600;cursor:pointer;}
.current-time{text-align:center; font-size:18px; font-weight:bold; color:#007bff; margin-bottom:20px; font-family:monospace;}
.nav-link{display:block;text-align:center;margin-top:20px;color:#007bff;text-decoration:none;}
.note{font-size:12px; color:#666; margin-top:-5px; margin-bottom:10px;}
</style></head><body><div class="container"><h2>Time Configuration</h2>
<div class="current-time">Device Time:<br>)rawliteral" + getTimestamp() + R"rawliteral(</div>

<div class="group">
<h3>1. Timezone Settings</h3>
<p class="note">Set this for both Internet Sync and Manual Time.</p>
<form onsubmit="return saveZone()">
<label>GMT Offset (Seconds)</label>
<select id="offset">
<option value="0">UTC +0</option>
<option value="28800">UTC +8 (Malaysia/China)</option>
<option value="19800">UTC +5:30 (India)</option>
<option value="3600">UTC +1 (Europe)</option>
<option value="-18000">UTC -5 (US East)</option>
<option value="custom">Custom...</option>
</select>
<input type="number" id="custom_offset" placeholder="Enter offset in seconds" style="display:none;">
<button type="submit">Save Timezone</button>
</form>
</div>

<div class="group">
<h3>2. Manual Time Set</h3>
<p class="note">Use this if the device has No Internet connection.</p>
<label>Set Date & Time</label>
<input type="datetime-local" id="manualTime">
<button onclick="setManualTime()">Update Device Clock</button>
</div>

<a href="/settings" class="nav-link">&larr; Back to Settings</a>
</div>
<script>
window.onload = function() {
    let currentOff = )rawliteral" + String(gmtOffset_sec) + R"rawliteral(;
    let sel = document.getElementById('offset');
    let found = false;
    for(let i=0; i<sel.options.length; i++){ if(sel.options[i].value == currentOff){ sel.selectedIndex = i; found=true; break; }}
    if(!found){ sel.value='custom'; document.getElementById('custom_offset').style.display='block'; document.getElementById('custom_offset').value = currentOff; }

    sel.addEventListener('change', function() {
        if(this.value === 'custom') document.getElementById('custom_offset').style.display = 'block';
        else document.getElementById('custom_offset').style.display = 'none';
    });

    const now = new Date();
    // Adjust to local time for the input picker
    now.setMinutes(now.getMinutes() - now.getTimezoneOffset());
    document.getElementById('manualTime').value = now.toISOString().slice(0,16);
};

function saveZone() {
    let off = document.getElementById('offset').value;
    if(off === 'custom') off = document.getElementById('custom_offset').value;
    fetch('/save_time?offset=' + off).then(() => { alert('Timezone Saved. Rebooting...'); });
    return false;
}

function setManualTime() {
    const val = document.getElementById('manualTime').value;
    if(!val) return;
    // Get timestamp in seconds
    const ts = Math.floor(new Date(val).getTime() / 1000);
    fetch('/set_manual_time?ts=' + ts).then(() => { alert('Clock Updated!'); location.reload(); });
}
</script></body></html>)rawliteral";
    server.send(200, "text/html", html);
}

void handleSaveTimeSettings() {
    long offset = server.arg("offset").toInt();
    preferences.begin("device-cfg", false);
    preferences.putLong("tz_offset", offset);
    preferences.end();
    server.send(200, "text/plain", "Saved.");
    delay(500); ESP.restart();
}

void handleSetManualTime() {
    long ts = server.arg("ts").toInt();
    struct timeval tv;
    tv.tv_sec = ts;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
    logMessage("Manual Time Set to timestamp: " + String(ts));
    server.send(200, "text/plain", "OK");
}

void handleClearSystemLog() {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        globalLogBuffer = ""; // Clear the string buffer
        String ts = getTimestamp();
        globalLogBuffer = ts + " [System] Log Cleared by User\n";
        xSemaphoreGive(dataMutex);
    }
    server.send(200, "text/plain", "System log cleared.");
}

void handleDeviceInfo() {
    // 1. Check if JSON format is requested for dynamic updates
    if (server.hasArg("format") && server.arg("format") == "json") {
        DynamicJsonDocument doc(1024);
        
        bool internetConnected = (WiFi.status() == WL_CONNECTED);
        String ipAddr = internetConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
        String rssiVal = internetConnected ? String(WiFi.RSSI()) + " dBm" : "N/A";
        String statusText = internetConnected ? "Connected to Internet" : "No Internet (AP Mode)";
        
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            doc["uptime"] = formatUptime(millis());
            doc["ram"] = String(ESP.getFreeHeap()) + " bytes";
            doc["log"] = globalLogBuffer;
            xSemaphoreGive(dataMutex);
        } else {
            doc["uptime"] = "--";
            doc["ram"] = "--";
            doc["log"] = "Log busy...";
        }
        
        doc["time"] = getTimestamp();
        doc["ip"] = ipAddr;
        doc["rssi"] = rssiVal;
        doc["status"] = statusText;
        doc["connected"] = internetConnected;

        String payload;
        serializeJson(doc, payload);
        server.send(200, "application/json", payload);
        return;
    }

    // 2. Otherwise serve the HTML page
    String uptime = formatUptime(millis());
    String localLog;
    String d_model, d_serial, d_url, d_pn, d_fw;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        localLog = globalLogBuffer;
        d_model = sensor_model;
        d_serial = sensor_serial;
        d_url = sensor_vendor_url; 
        d_pn = sensor_part_number;
        d_fw = sensor_fw_version;
        xSemaphoreGive(dataMutex);
    }
    localLog.replace("\n", "<br>");

    bool internetConnected = (WiFi.status() == WL_CONNECTED);
    String ipAddr = internetConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    String rssiVal = internetConnected ? String(WiFi.RSSI()) + " dBm" : "N/A";

    String html = R"rawliteral(
<!DOCTYPE html><html><head><title>Device Diagnostics</title><meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://unpkg.com/lucide@latest"></script>
<style>
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;margin:0;background-color:#f4f6f8;display:flex;justify-content:center;padding:20px 0;}
.container{width:100%;max-width:800px;background-color:white;border-radius:12px;box-shadow:0 8px 20px rgba(0,0,0,.1);text-align:center;padding:30px}
h2{color:#2c3e50;margin-top:0}.info-grid{display:grid;grid-template-columns:1fr 1fr;gap:15px;text-align:left;margin-top:25px}
.info-item{background-color:#f8f9fa;padding:15px;border-radius:8px}
.info-label{font-size:12px;color:#6c757d;font-weight:600;display:block;margin-bottom:5px;text-transform:uppercase;letter-spacing:0.5px;}
.info-value{font-size:16px;color:#343a40;font-weight:500; word-wrap: break-word;}
.full-width { grid-column: 1 / -1; }
.log-box { margin-top: 25px; text-align: left; }
.log-header { display:flex; justify-content:space-between; align-items:center; margin-bottom: 10px; }
.log-content { background-color: #2c3e50; color: #a4b0be; border: 1px solid #e9ecef; border-radius: 8px; padding: 15px; max-height: 400px; overflow-y: auto; font-family: "Consolas", "Monaco", monospace; font-size: 12px; white-space: pre-wrap; line-height: 1.4; }
.btn-clear { background-color: #e74c3c; color: white; border: none; padding: 6px 12px; border-radius: 4px; cursor: pointer; font-size: 12px; display: flex; align-items: center; gap: 5px; }
.btn-clear:hover { background-color: #c0392b; }
.status-connected { color: #2ecc71; font-weight: bold; }
.status-disconnected { color: #e74c3c; font-weight: bold; }
.nav-link{display:inline-block;margin-top:30px;color:#007bff;text-decoration:none;font-weight:600; border: 1px solid #007bff; padding: 10px 20px; border-radius: 6px; transition: all 0.2s; }
.nav-link:hover { background-color: #007bff; color: white; }

/* OTA Section Styles */
.ota-box { background: #eef2f5; border: 1px solid #dee2e6; border-radius: 8px; padding: 20px; margin-top: 25px; text-align: left; }
.ota-title { font-weight: 600; color: #2c3e50; margin-bottom: 15px; display: flex; align-items: center; gap: 8px; }
.file-upload-row { display: flex; gap: 10px; align-items: center; margin-bottom: 15px; }
input[type=file] { background: white; padding: 8px; border-radius: 6px; border: 1px solid #ced4da; flex-grow: 1; }
.btn-upload { background-color: #007bff; color: white; border: none; padding: 10px 20px; border-radius: 6px; cursor: pointer; font-weight: 600; }
.btn-upload:disabled { background-color: #6c757d; cursor: not-allowed; }
.progress-container { width: 100%; background-color: #dfe6e9; border-radius: 10px; height: 20px; overflow: hidden; display: none; margin-bottom: 10px; }
.progress-bar { width: 0%; height: 100%; background-color: #2ecc71; text-align: center; line-height: 20px; color: white; font-size: 12px; font-weight: bold; transition: width 0.3s; }
#ota-status { font-size: 14px; font-weight: 500; min-height: 20px; }
.status-error { color: #dc3545; }
.status-success { color: #28a745; }
.ota-drop-zone { border: 2px dashed #ced4da; border-radius: 8px; padding: 30px; text-align: center; color: #6c757d; cursor: pointer; transition: 0.3s; background: white; }
.ota-drop-zone.active { border-color: #007bff; background-color: #f1f8ff; color: #007bff; }
.ota-drop-zone p { margin: 10px 0 0; font-size: 14px; }
</style></head><body><div class="container"><h2>Device Diagnostics</h2>
<div class="info-grid">

<div class="info-item"><span class="info-label">Sensor Model</span><span class="info-value">)rawliteral" + d_model + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">Part Number</span><span class="info-value">)rawliteral" + d_pn + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">Sensor Serial</span><span class="info-value">)rawliteral" + d_serial + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">Vendor URL</span><span class="info-value">)rawliteral" + d_url + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">Sensor Firmware</span><span class="info-value">)rawliteral" + d_fw + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">Connection Status</span><span class="info-value" id="val-status">)rawliteral" + (internetConnected ? "Connected to Internet" : "No Internet (AP Mode)") + R"rawliteral(</span></div>

<div class="info-item"><span class="info-label">Last Reset Reason</span><span class="info-value" style="color:#d63031">)rawliteral" + lastResetReason + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">IP Address</span><span class="info-value" id="val-ip">)rawliteral" + ipAddr + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">Signal Strength (RSSI)</span><span class="info-value" id="val-rssi">)rawliteral" + rssiVal + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">System Time</span><span class="info-value" id="val-time">)rawliteral" + getTimestamp() + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">System Uptime</span><span class="info-value" id="val-uptime">)rawliteral" + uptime + R"rawliteral(</span></div>
<div class="info-item"><span class="info-label">Free RAM</span><span class="info-value" id="val-ram">)rawliteral" + String(ESP.getFreeHeap()) + R"rawliteral( bytes</span></div>
<div class="info-item"><span class="info-label">Firmware</span><span class="info-value">)rawliteral" + String(FIRMWARE_VERSION) + R"rawliteral(</span></div>
</div>

<div class="ota-box">
    <div class="ota-title"><i data-lucide="upload-cloud"></i> Firmware Update (OTA)</div>
    <div class="ota-drop-zone" id="dropZone" onclick="document.getElementById('otafile').click()">
        <i data-lucide="file-up" style="width: 32px; height: 32px;"></i>
        <p>Click or Drag & Drop <strong>.bin</strong> firmware file here</p>
        <div id="fileName" style="margin-top: 10px; font-weight: bold; color: #333; display:none;"></div>
    </div>
    <input type="file" id="otafile" name="update" accept=".bin" style="display:none" onchange="fileSelected(this)">
    <button class="btn-upload" onclick="performOTA()" id="uploadBtn" style="width:100%; margin-top: 15px;" disabled>Start Update</button>
    <div class="progress-container" id="progressWrap">
        <div class="progress-bar" id="progressBar">0%</div>
    </div>
    <div id="ota-status"></div>
</div>

<div class="log-box">
  <div class="log-header">
    <h3>System Event Log</h3>
    <button class="btn-clear" onclick="clearLog()"><i data-lucide="trash-2" style="width:14px;"></i> Clear Log</button>
  </div>
  <div class="log-content" id="logContent">)rawliteral" + localLog + R"rawliteral(</div>
</div>
<a href="/settings" class="nav-link">&larr; Back to Settings</a></div>
<script>
if (typeof lucide !== 'undefined') { lucide.createIcons(); }

// --- Live Update Logic ---
setInterval(updateDiagnostics, 2000);

function updateDiagnostics() {
    fetch('/device_info?format=json')
        .then(response => response.json())
        .then(data => {
            document.getElementById('val-uptime').textContent = data.uptime;
            document.getElementById('val-ram').textContent = data.ram;
            document.getElementById('val-time').textContent = data.time;
            document.getElementById('val-ip').textContent = data.ip;
            document.getElementById('val-rssi').textContent = data.rssi;
            
            const statusEl = document.getElementById('val-status');
            statusEl.textContent = data.status;
            if(data.connected) {
                statusEl.classList.remove('status-disconnected');
                statusEl.classList.add('status-connected');
            } else {
                statusEl.classList.remove('status-connected');
                statusEl.classList.add('status-disconnected');
            }
            
            // Optional: Update Log if you want real-time logs
            // document.getElementById('logContent').innerHTML = data.log.replace(/\n/g, '<br>');
        })
        .catch(err => console.error('Error fetching diagnostics:', err));
}

function clearLog() {
    if(confirm("Clear system event log?")) {
        fetch('/clear_system_log').then(r => r.text()).then(msg => {
            document.getElementById('logContent').innerHTML = "Log cleared.";
        });
    }
}

// --- Drag & Drop Logic ---
const dropZone = document.getElementById('dropZone');
const fileInput = document.getElementById('otafile');

['dragenter', 'dragover', 'dragleave', 'drop'].forEach(eventName => {
  dropZone.addEventListener(eventName, preventDefaults, false);
});
function preventDefaults(e) { e.preventDefault(); e.stopPropagation(); }

['dragenter', 'dragover'].forEach(eventName => {
  dropZone.addEventListener(eventName, () => dropZone.classList.add('active'), false);
});
['dragleave', 'drop'].forEach(eventName => {
  dropZone.addEventListener(eventName, () => dropZone.classList.remove('active'), false);
});

dropZone.addEventListener('drop', handleDrop, false);

function handleDrop(e) {
  const dt = e.dataTransfer;
  const files = dt.files;
  if(files.length > 0) {
      fileInput.files = files;
      fileSelected(fileInput);
  }
}

function fileSelected(input) {
    const file = input.files[0];
    if (file) {
        if (!file.name.endsWith(".bin")) {
            alert("Invalid file type! Please select a .bin firmware file.");
            input.value = ""; // Clear
            document.getElementById('fileName').style.display = 'none';
            document.getElementById('uploadBtn').disabled = true;
            return;
        }
        document.getElementById('fileName').textContent = file.name + " (" + (file.size/1024/1024).toFixed(2) + " MB)";
        document.getElementById('fileName').style.display = 'block';
        document.getElementById('uploadBtn').disabled = false;
        document.getElementById('ota-status').innerHTML = '';
    }
}

function performOTA() {
    var file = fileInput.files[0];
    if (!file) return;

    if(!confirm("Start firmware update? Do not power off the device.")) return;

    var status = document.getElementById('ota-status');
    var btn = document.getElementById('uploadBtn');
    var progressBar = document.getElementById('progressBar');
    var progressWrap = document.getElementById('progressWrap');
    
    // Warn user before leaving page
    window.onbeforeunload = function() { return "Update in progress. Are you sure?"; };

    var formData = new FormData();
    formData.append("update", file);
    var xhr = new XMLHttpRequest();
    
    // UI Reset
    btn.disabled = true;
    btn.textContent = "Uploading...";
    status.innerHTML = '<span style="color:#007bff">Starting upload...</span>';
    progressWrap.style.display = 'block';
    progressBar.style.width = '0%';
    progressBar.innerHTML = '0%';

    // Progress Handler
    xhr.upload.addEventListener("progress", function(e) {
        if (e.lengthComputable) {
            var percent = Math.round((e.loaded / e.total) * 100);
            var loadedMB = (e.loaded / 1024 / 1024).toFixed(2);
            var totalMB = (e.total / 1024 / 1024).toFixed(2);
            
            progressBar.style.width = percent + '%';
            progressBar.innerHTML = percent + '%';
            status.innerHTML = `Uploading: ${loadedMB} MB / ${totalMB} MB`;
            
            if(percent === 100) status.innerHTML = '<span style="color:#28a745">Finalizing update... please wait.</span>';
        }
    }, false);

    // Completion Handler
    xhr.onload = function() {
        window.onbeforeunload = null; // Remove warning
        if (xhr.status === 200) {
            status.innerHTML = '<span class="status-success">Update Success! Rebooting...</span>';
            progressBar.style.backgroundColor = '#2ecc71';
            setTimeout(function() { window.location.href = '/'; }, 10000);
        } else {
            status.innerHTML = '<span class="status-error">Update Failed! Server Error.</span>';
            btn.disabled = false;
            btn.textContent = "Retry Upload";
        }
    };

    // Error Handler
    xhr.onerror = function() {
        window.onbeforeunload = null;
        status.innerHTML = '<span class="status-error">Update Failed! Network Error.</span>';
        btn.disabled = false;
        btn.textContent = "Retry Upload";
    };

    xhr.open("POST", "/update");
    xhr.send(formData);
}
</script>
</body></html>)rawliteral";
    server.send(200, "text/html", html);
}

void handleModbusTcpSettings() {
    String ipAddress = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Modbus TCP Server</title>
    <style>
        :root { --primary: #0056b3; --secondary: #6c757d; --bg: #f8f9fa; --border: #dee2e6; --green: #28a745; --red: #dc3545; }
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background-color: var(--bg); color: #333; margin: 0; padding: 20px; }
        .container { max-width: 1000px; margin: 0 auto; background: white; padding: 30px; border-radius: 12px; box-shadow: 0 4px 20px rgba(0,0,0,0.05); }
        h1 { margin-top: 0; color: #2c3e50; border-bottom: 2px solid var(--border); padding-bottom: 15px; display: flex; justify-content: space-between; align-items: center; }
        .config-box { background: #e9ecef; padding: 20px; border-radius: 8px; margin-bottom: 30px; border-left: 5px solid var(--primary); }
        .form-row { display: flex; align-items: center; justify-content: space-between; margin-bottom: 15px; }
        .info-row { display: flex; gap: 20px; margin-bottom: 20px; font-family: monospace; font-size: 1.1em; background: #fff; padding: 10px; border-radius: 6px; }
        label { font-weight: 600; color: #495057; }
        input[type=number] { padding: 8px; border-radius: 4px; border: 1px solid #ced4da; width: 80px; }
        button { background: var(--primary); color: white; border: none; padding: 10px 20px; border-radius: 6px; cursor: pointer; font-weight: 600; }
        button:hover { background: #004494; }
        
        /* Toggle Switch */
        .switch { position: relative; display: inline-block; width: 50px; height: 26px; }
        .switch input { opacity: 0; width: 0; height: 0; }
        .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }
        .slider:before { position: absolute; content: ""; height: 20px; width: 20px; left: 3px; bottom: 3px; background-color: white; transition: .4s; border-radius: 50%; }
        input:checked + .slider { background-color: var(--green); }
        input:checked + .slider:before { transform: translateX(24px); }

        /* Tables */
        table { width: 100%; border-collapse: collapse; margin-top: 10px; font-size: 0.9em; margin-bottom: 30px; }
        th, td { border: 1px solid var(--border); padding: 8px 12px; text-align: left; vertical-align: top; }
        th { background-color: #f1f3f5; font-weight: 700; position: sticky; top: 0; }
        
        .addr-badge { background: #495057; color: white; padding: 2px 6px; border-radius: 4px; font-family: monospace; font-size: 0.9em; }
        .live-val { font-family: monospace; font-weight: bold; color: var(--primary); }
        .section-header { background-color: #343a40; color: white; padding: 10px; font-size: 1.1em; border-radius: 6px 6px 0 0; margin-bottom: 0; }
        .section-desc { font-size: 0.9em; color: #666; margin-bottom: 10px; padding: 0 5px; }
        
        .badge { padding: 4px 8px; border-radius: 4px; color: white; font-size: 0.8em; font-weight: bold; }
        .bg-green { background-color: var(--green); }
        .bg-blue { background-color: #17a2b8; }
        .axis-header { background-color: #e2e6ea; font-weight: bold; text-align: center; }
        .source-map { font-size: 0.85em; color: #555; margin-top: 5px; line-height: 1.4; border-top: 1px dashed #ccc; padding-top: 4px; }
    </style>
</head>
<body>
<div class="container">
    <h1>Modbus TCP Server</h1>
    
    <div class="config-box">
        <div class="info-row">
            <span><strong>IP Address:</strong> <span style="color:var(--primary)">)rawliteral" + ipAddress + R"rawliteral(</span></span>
            <span><strong>Status:</strong> <span id="server-status" style="font-weight:bold;">)rawliteral" + String(modbusTcpEnabled ? "RUNNING" : "STOPPED") + R"rawliteral(</span></span>
            <span><strong>Slave ID:</strong> 1</span>
        </div>
        
        <form onsubmit="return saveSettings()">
            <div class="form-row">
                <label for="enabled">Enable Server</label>
                <label class="switch"><input type="checkbox" id="enabled" name="enabled"><span class="slider"></span></label>
            </div>
            <div class="form-row">
                <label for="port">Port (Default: 502)</label>
                <input type="number" id="port" name="port" min="1" max="65535" required>
            </div>
            <button type="submit">Save Configuration & Reboot</button>
        </form>
    </div>

    <div class="section-header">Live Metrics Map (Read Only)</div>
    <p class="section-desc">Registers 0-60 update in real-time based on sensor polling.</p>
    <table>
        <thead><tr><th>Address</th><th>Metric Name</th><th>Unit</th><th>Live Value</th></tr></thead>
        <tbody id="live-table-body"></tbody>
    </table>

    <div class="section-header">Section 1: Control & Status (Read/Write)</div>
    <p class="section-desc">Trigger captures and check status.</p>
    <table>
        <thead><tr><th>Addr</th><th>Name</th><th>R/W</th><th>Description</th></tr></thead>
        <tbody>
            <tr>
                <td><span class="addr-badge">90</span></td>
                <td>Waveform Status</td>
                <td>R</td>
                <td>0=Busy, 2=X Ready, 3=Y Ready, 4=Z Ready</td>
            </tr>
            <tr>
                <td><span class="addr-badge">91</span></td>
                <td>Spectrum Status</td>
                <td>R</td>
                <td>0=Busy, 2=X Ready, 3=Y Ready, 4=Z Ready</td>
            </tr>
            <tr>
                <td><span class="addr-badge">92</span></td>
                <td>Trigger Waveform</td>
                <td>W</td>
                <td>Write: 1=All(XYZ), 2=X Only, 3=Y Only, 4=Z Only</td>
            </tr>
            <tr>
                <td><span class="addr-badge">93</span></td>
                <td>Trigger Spectrum</td>
                <td>W</td>
                <td>Write: 1=All(XYZ), 2=X Only, 3=Y Only, 4=Z Only</td>
            </tr>
        </tbody>
    </table>

    <div class="section-header">Section 2: Time Waveform Data (Read Only)</div>
    <p class="section-desc"><strong>Range:</strong> 1000 to 14333. <strong>Type:</strong> Signed Int16 (x 0.001 g). Data updates when Status (90) changes.</p>
    <table>
        <thead><tr><th>Block #</th><th>Start</th><th>End</th><th>Length</th><th>Description</th></tr></thead>
        <tbody>
            <tr><td>1</td><td><span class="addr-badge">1000</span></td><td>1099</td><td>100</td><td>Points 1 - 100</td></tr>
            <tr><td>2</td><td><span class="addr-badge">1100</span></td><td>1199</td><td>100</td><td>Points 101 - 200</td></tr>
            <tr><td>...</td><td>...</td><td>...</td><td>...</td><td>...</td></tr>
            <tr><td>134</td><td><span class="addr-badge">14300</span></td><td>14333</td><td>34</td><td>Points 13301 - 13334</td></tr>
        </tbody>
    </table>

    <div class="section-header">Section 3: FFT Spectrum Data (Read Only)</div>
    <p class="section-desc"><strong>Range:</strong> 20000 to 26144. <strong>Type:</strong> Unsigned Int16 (x 0.001 g). Data updates when Status (91) changes.</p>
    <table>
        <thead><tr><th>Block #</th><th>Start</th><th>End</th><th>Length</th><th>Description</th></tr></thead>
        <tbody>
            <tr><td>1</td><td><span class="addr-badge">20000</span></td><td>20099</td><td>100</td><td>Points 1 - 100</td></tr>
            <tr><td>2</td><td><span class="addr-badge">20100</span></td><td>20199</td><td>100</td><td>Points 101 - 200</td></tr>
            <tr><td>...</td><td>...</td><td>...</td><td>...</td><td>...</td></tr>
            <tr><td>62</td><td><span class="addr-badge">26100</span></td><td>26144</td><td>45</td><td>Points 6101 - 6145</td></tr>
        </tbody>
    </table>

    <div class="config-box" style="border-left-color: #28a745;">
        <h3>Example Usage (PLC / Node-RED)</h3>
        <p><strong>Scenario: Manual X-Axis Capture</strong></p>
        <ol>
            <li><strong>Write:</strong> Value <code>2</code> to Address <code>92</code> (Trigger Waveform).</li>
            <li><strong>Poll:</strong> Read Address <code>90</code>. Wait until it becomes <code>2</code> (Ready).</li>
            <li><strong>Read:</strong> Read Address <code>1000</code> (Length 100...) to get data.</li>
        </ol>
    </div>

    <div style="margin-top:30px; text-align:center;">
        <a href="/settings" style="display:inline-block; padding:10px 20px; background:#6c757d; color:white; text-decoration:none; border-radius:6px; font-weight:600;">&larr; Back to Settings</a>
    </div>
</div>

<script>
    const liveRegisters=[
        // X-Axis
        {addr:0,name:"X Accel RMS (2-1k)",unit:"g",key:"accelRms",axis:"x"}, 
        {addr:1,name:"X Accel Peak (2-1k)",unit:"g",key:"accelPeak",axis:"x"},
        {addr:2,name:"X Accel RMS (10-5k)",unit:"g",key:"accelRms2",axis:"x"}, 
        {addr:3,name:"X Accel Peak (10-5k)",unit:"g",key:"accelPeak2",axis:"x"},
        {addr:4,name:"X Vel RMS (10-1k)",unit:"mm/s",key:"veloRms",axis:"x"}, 
        {addr:5,name:"X Vel Peak (10-1k)",unit:"mm/s",key:"veloPeak",axis:"x"},
        {addr:6,name:"X Disp RMS (10-1k)",unit:"um",key:"displRms",axis:"x"}, 
        {addr:7,name:"X Disp Peak (10-1k)",unit:"um",key:"displPeak",axis:"x"},
        {addr:8,name:"X True Peak",unit:"g",key:"truePeak",axis:"x"}, 
        {addr:9,name:"X Crest Factor",unit:"-",key:"crestFactor",axis:"x"},
        {addr:10,name:"X Accel Metric RMS",unit:"m/s²",key:"accelMetricRms",axis:"x"},
        {addr:14,name:"X Vel Wide RMS",unit:"mm/s",key:"veloWideRms",axis:"x"},
        {addr:16,name:"X Disp Low RMS",unit:"um",key:"displLowRms",axis:"x"},

        // Y-Axis
        {addr:20,name:"Y Accel RMS (2-1k)",unit:"g",key:"accelRms",axis:"y"}, 
        {addr:21,name:"Y Accel Peak (2-1k)",unit:"g",key:"accelPeak",axis:"y"},
        {addr:22,name:"Y Accel RMS (10-5k)",unit:"g",key:"accelRms2",axis:"y"},
        {addr:24,name:"Y Vel RMS (10-1k)",unit:"mm/s",key:"veloRms",axis:"y"},
        {addr:28,name:"Y True Peak",unit:"g",key:"truePeak",axis:"y"},
        {addr:29,name:"Y Crest Factor",unit:"-",key:"crestFactor",axis:"y"},

        // Z-Axis
        {addr:40,name:"Z Accel RMS (2-1k)",unit:"g",key:"accelRms",axis:"z"}, 
        {addr:41,name:"Z Accel Peak (2-1k)",unit:"g",key:"accelPeak",axis:"z"},
        {addr:42,name:"Z Accel RMS (10-5k)",unit:"g",key:"accelRms2",axis:"z"},
        {addr:44,name:"Z Vel RMS (10-1k)",unit:"mm/s",key:"veloRms",axis:"z"},
        {addr:48,name:"Z True Peak",unit:"g",key:"truePeak",axis:"z"},
        {addr:49,name:"Z Crest Factor",unit:"-",key:"crestFactor",axis:"z"},
        
        // System
        {addr:60,name:"Temperature",unit:"C",key:"temperature",axis:null},
        {addr:90,name:"Waveform Ready",unit:"Bool",key:"newWaveformAvailable",axis:null},
        {addr:91,name:"Spectrum Ready",unit:"Bool",key:"newSpectrumAvailable",axis:null}
    ];

    function generateLiveTable(){
        let h=''; 
        liveRegisters.forEach(r=>{
            let id = `val-${r.key}${r.axis ? '-'+r.axis : ''}`;
            // Add spacing rows for visual separation of axes
            if(r.addr === 20) h += '<tr class="axis-header"><td colspan="4">Y-AXIS</td></tr>';
            if(r.addr === 40) h += '<tr class="axis-header"><td colspan="4">Z-AXIS</td></tr>';
            if(r.addr === 60) h += '<tr class="axis-header"><td colspan="4">SYSTEM</td></tr>';
            
            h+=`<tr>
                <td><span class="addr-badge">${r.addr}</span></td>
                <td>${r.name}</td>
                <td>${r.unit}</td>
                <td class="live-val" id="${id}">--</td>
            </tr>`
        });
        document.getElementById('live-table-body').innerHTML=h;
    }

    function updateLiveValues(){
        fetch('/data').then(r=>r.json()).then(d=>{
            liveRegisters.forEach(r=>{
                let v = (r.axis) ? d[r.key][r.axis] : d[r.key];
                let el = document.getElementById(`val-${r.key}${r.axis ? '-'+r.axis : ''}`);
                if(el) {
                    if(typeof v === 'boolean') el.textContent = v ? "1 (YES)" : "0 (NO)";
                    else if(typeof v === 'number') el.textContent = v.toFixed(3);
                    else el.textContent = "--";
                }
            });
        });
    }

    function saveSettings(){
        const e = document.getElementById('enabled').checked;
        const p = document.getElementById('port').value;
        if(confirm("Save settings and reboot device?")) {
            window.location.href=`/save_modbustcp?enabled=${e}&port=${p}`;
        }
        return false;
    }

    window.onload=function(){
        document.getElementById('enabled').checked = )rawliteral" + String(modbusTcpEnabled ? "true" : "false") + R"rawliteral(;
        document.getElementById('port').value = )rawliteral" + String(modbusTcpPort) + R"rawliteral(;
        
        generateLiveTable();
        setInterval(updateLiveValues, 2000); // Poll every 2s
        updateLiveValues();
    };
</script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

void handleSaveModbusTcp() {
    preferences.begin("modbustcp-cfg", false); 
    bool enabled = (server.arg("enabled") == "true");
    uint16_t port = server.arg("port").toInt();
    preferences.putBool("enabled", enabled);
    preferences.putUShort("port", port);
    preferences.end();
    server.send(200, "text/plain", "Settings saved. Rebooting now...");
    delay(1000);
    ESP.restart();
}

// #include <WiFi.h>  // Commented out to save space
// #include <ArduinoJson.h> // Commented out to save space
#include <Wire.h> // Required for I2C communication with MPU-6050
#include <math.h> // Required for mathematical functions like atan2 and sqrt
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <BleMouse.h> // Required for BLE mouse functionality

// Edge Impulse inferencing library
#include <minerfrands-project-1_inferencing.h>

// --- WiFi Credentials (Removed to save space) ---
// const char *ssid = "Teehee";
// const char *password = "7eb3a6e4";

// --- AirMouse Configuration ---
const float MOUSE_SENSITIVITY_X = 1.0;  // Adjust for horizontal sensitivity
const float MOUSE_SENSITIVITY_Y = 0.65; // Adjust for vertical sensitivity
const int TOUCH_THRESHOLD = 40;         // Threshold for touch detection
const int TOUCH_PIN_NUMBER = T3;        // GPIO15 is TOUCH_CHANNEL 3
const int RESTART_PIN = 25;             // Pin for restart button

// --- Haptic Feedback Configuration ---
const int HAPTIC_PIN = 13;       // vibration-motor FET / driver
const int HAPTIC_VOLTAGE = HIGH; // max-voltage pulse

struct HapticPattern
{
    uint8_t pulses;   // number of pulses
    uint16_t onTime;  // ms motor ON   (≥ 350 ms)
    uint16_t offTime; // ms motor OFF between pulses
};

// polished patterns – every pulse ≥ 350 ms
const HapticPattern HAPTIC_CLICK = {1, 350, 0};
const HapticPattern HAPTIC_DOUBLE_CLICK = {2, 350, 80};
const HapticPattern HAPTIC_LONG_PRESS = {1, 500, 0};
const HapticPattern HAPTIC_GESTURE_OK = {3, 350, 80};
const HapticPattern HAPTIC_MODE_SWITCH = {2, 400, 150};
const HapticPattern HAPTIC_ERROR = {4, 350, 120};
const HapticPattern HAPTIC_CONNECT = {1, 600, 0};
const HapticPattern HAPTIC_DRAG_START = {2, 350, 60};
const HapticPattern HAPTIC_DRAG_END = {1, 350, 0};

// Click detection variables
unsigned long firstClickTime = 0;
int lastButtonState = HIGH;
const unsigned long DOUBLE_CLICK_TIMEOUT = 500; // Time in ms to wait for a second click
const unsigned long LONG_PRESS_THRESHOLD = 250; // Time in ms for a long press (for click and drag)
bool isDragging = false;

// Improved click state
int clickCount = 0;
unsigned long lastClickTime = 0;

// BLE Mouse object
BleMouse bleMouse("AirMouse", "ESP32", 100);

// --- Gesture Detection Variables ---
volatile char latestGesture[32] = "";
volatile float latestConfidence = 0.0;
volatile bool newGestureAvailable = false;
volatile bool gestureDetectionEnabled = false; // Only enabled when NOT oriented

// --- Orientation tracking for mode switching ---
bool previouslyOriented = false;
unsigned long lastModeSwitch = 0;
const unsigned long MODE_SWITCH_COOLDOWN = 1000; // Prevent rapid mode switching

// --- Sensor Data Variables ---
float currentAccelX = 0.0;
float currentAccelY = 0.0;
float currentAccelZ = 0.0;
float currentGyroX = 0.0;
float currentGyroY = 0.0;
float currentGyroZ = 0.0;

// --- MPU-6050 Pin Configuration ---
const int MPU_SDA_PIN = 23;
const int MPU_SCL_PIN = 19;
const int MPU_GND_PIN = 22; // Using GPIO 22 as GND

uint8_t i2cData[14]; // Buffer for I2C data

const uint8_t IMUAddress = 0x68;   // MPU-6050 I2C address
const uint16_t I2C_TIMEOUT = 1000; // I2C timeout in microseconds

// Global features array for Edge Impulse (reduced size if possible)
static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Sensor data variables
int16_t accelX, accelY, accelZ;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

// Sensor offsets (to be calibrated)
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;
float accelX_offset = 0;
float accelY_offset = 0;

// Complementary filter variables for AirMouse
const float alpha = 0.96;     // Weight for gyroscope data
float angleX = 0, angleY = 0; // Fused angles

unsigned long last_read_time;
float dt;

// Task handles
TaskHandle_t TensorFlowTaskHandle = NULL;
TaskHandle_t AirMouseTaskHandle = NULL;
TaskHandle_t HapticTaskHandle;

// Mutex for shared sensor data
SemaphoreHandle_t sensorDataMutex;
QueueHandle_t hapticQueue;

// Haptic feedback functions
void hapticFeedback(const HapticPattern &pat)
{
    if (hapticQueue)
        xQueueSend(hapticQueue, &pat, 0);
}

void hapticTask(void *pv)
{
    HapticPattern pat;
    pinMode(HAPTIC_PIN, OUTPUT);
    digitalWrite(HAPTIC_PIN, LOW);

    for (;;)
    {
        if (xQueueReceive(hapticQueue, &pat, portMAX_DELAY) == pdTRUE)
        {
            for (uint8_t i = 0; i < pat.pulses; ++i)
            {
                digitalWrite(HAPTIC_PIN, HAPTIC_VOLTAGE);
                vTaskDelay(pdMS_TO_TICKS(pat.onTime));
                digitalWrite(HAPTIC_PIN, LOW);
                if (i < pat.pulses - 1)
                    vTaskDelay(pdMS_TO_TICKS(pat.offTime));
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println("ESP32 Dual-Core AirMouse + TensorFlow with Haptic Feedback");

    // --- Initialize MPU-6050 ---
    pinMode(MPU_GND_PIN, OUTPUT);
    digitalWrite(MPU_GND_PIN, LOW);

    // Initialize haptic feedback pin
    pinMode(HAPTIC_PIN, OUTPUT);
    digitalWrite(HAPTIC_PIN, LOW);

    // Initialize restart button (moved to different pin since 12 is now haptic)
    pinMode(RESTART_PIN, INPUT_PULLUP);
    touchAttachInterrupt(TOUCH_PIN_NUMBER, NULL, TOUCH_THRESHOLD);

    Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);

    // MPU-6050 initialization
    i2cData[0] = 7;
    i2cData[1] = 0x00;
    i2cData[3] = 0x00;

    while (i2cWrite(0x19, i2cData, 4, false))
        ;
    while (i2cWrite2(0x6B, 0x01, true))
        ;
    while (i2cRead(0x75, i2cData, 1))
        ;
    delay(100);
    while (i2cRead(0x3B, i2cData, 6))
        ;

    Serial.println("Calibrating... Keep still.");
    calibrateSensors();
    Serial.println("Calibration done.");

    // Startup haptic feedback
    hapticFeedback(HAPTIC_CONNECT);

    last_read_time = micros();

    Serial.println("Starting BLE Mouse");
    bleMouse.begin();
    delay(100);

    sensorDataMutex = xSemaphoreCreateMutex();

    // Create tasks
    hapticQueue = xQueueCreate(8, sizeof(HapticPattern));
    xTaskCreatePinnedToCore(hapticTask,
                            "Haptic_Task",
                            2048,
                            nullptr,
                            2, // priority higher than mouse but lower than Wi-Fi/BT
                            &HapticTaskHandle,
                            1); // run on same core as Air-Mouse

    xTaskCreatePinnedToCore(tensorFlowTask, "TF_Task", 8000, NULL, 1, &TensorFlowTaskHandle, 0);
    xTaskCreatePinnedToCore(airMouseTask, "Mouse_Task", 6000, NULL, 1, &AirMouseTaskHandle, 1);

    Serial.println("Tasks created - Core 0: TensorFlow, Core 1: AirMouse");
    Serial.println("Gesture controls: Circle=Back, Up/Down=Forward (only when not oriented)");
    Serial.println("Haptic feedback enabled on pin 13");
}

// TensorFlow inference task running on Core 0
void tensorFlowTask(void *pvParameters)
{
    static size_t feature_idx = 0;

    for (;;)
    {
        if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY))
        {
            readSensorData();
            xSemaphoreGive(sensorDataMutex);
        }

        if (feature_idx < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
        {
            features[feature_idx++] = currentAccelX;
            features[feature_idx++] = currentAccelY;
            features[feature_idx++] = currentAccelZ;
            features[feature_idx++] = currentGyroX;
            features[feature_idx++] = currentGyroY;
            features[feature_idx++] = currentGyroZ;
        }

        if (feature_idx == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
        {
            // Only run inference if gesture detection is enabled (when NOT oriented)
            if (gestureDetectionEnabled)
            {
                signal_t signal;
                signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
                signal.get_data = &raw_feature_get_data;

                ei_impulse_result_t result = {0};
                EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

                if (r == EI_IMPULSE_OK)
                {
                    // Find the highest confidence result
                    float max_confidence = 0;
                    int max_index = 0;
                    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
                    {
                        if (result.classification[ix].value > max_confidence)
                        {
                            max_confidence = result.classification[ix].value;
                            max_index = ix;
                        }
                    }

                    // Only process high confidence gestures
                    if (max_confidence > 0.7)
                    {
                        // Update shared gesture variables
                        strcpy((char *)latestGesture, result.classification[max_index].label);
                        latestConfidence = max_confidence;
                        newGestureAvailable = true;

                        Serial.print("Gesture: ");
                        Serial.print(result.classification[max_index].label);
                        Serial.print(" (");
                        Serial.print(max_confidence);
                        Serial.println(")");
                    }
                }
            }
            feature_idx = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(EI_CLASSIFIER_INTERVAL_MS));
    }
}

// AirMouse task running on Core 1
void airMouseTask(void *pvParameters)
{
    float calibratedGyroX, calibratedGyroY, calibratedGyroZ;
    float calibratedAccelX, calibratedAccelY;
    float mouseMoveX, mouseMoveY;
    float gyroRateX, gyroRateY;
    float accelAngleX, accelAngleY;
    bool isOriented = false;
    bool bleConnected = false;

    for (;;)
    {
        unsigned long current_time = micros();
        dt = (current_time - last_read_time) / 1000000.0;
        last_read_time = current_time;

        if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY))
        {
            calibratedGyroX = currentGyroX;
            calibratedGyroY = currentGyroY;
            calibratedGyroZ = currentGyroZ;
            calibratedAccelX = currentAccelX;
            calibratedAccelY = currentAccelY;
            xSemaphoreGive(sensorDataMutex);
        }

        // Sensor Fusion
        gyroRateX = calibratedGyroX / 131.0;
        gyroRateY = calibratedGyroY / 131.0;

        accelAngleX = atan2(calibratedAccelY, accelZ) * 180 / M_PI;
        accelAngleY = atan2(-calibratedAccelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / M_PI;

        angleX = alpha * (angleX + gyroRateX * dt) + (1 - alpha) * accelAngleX;
        angleY = alpha * (angleY + gyroRateY * dt) + (1 - alpha) * accelAngleY;

        // Check orientation
        isOriented = (accelAngleY > -20.0 && accelAngleY < 20.0);

        // Detect mode switch and provide haptic feedback
        if (isOriented != previouslyOriented && (millis() - lastModeSwitch > MODE_SWITCH_COOLDOWN))
        {
            hapticFeedback(HAPTIC_MODE_SWITCH);
            if (isOriented)
            {
                Serial.println("→ Mouse Mode Activated");
            }
            else
            {
                Serial.println("→ Gesture Mode Activated");
            }
            previouslyOriented = isOriented;
            lastModeSwitch = millis();
        }

        // Enable gesture detection only when NOT oriented
        gestureDetectionEnabled = !isOriented;

        if (digitalRead(RESTART_PIN) == LOW)
        {
            hapticFeedback(HAPTIC_ERROR);
            ESP.restart();
        }

        // Check BLE connection status for haptic feedback
        if (bleMouse.isConnected() != bleConnected)
        {
            bleConnected = bleMouse.isConnected();
            if (bleConnected)
            {
                hapticFeedback(HAPTIC_CONNECT);
                Serial.println("BLE Connected - Haptic confirmation");
            }
        }

        if (bleMouse.isConnected())
        {
            if (isOriented)
            {
                // ORIENTED MODE - Normal mouse control
                mouseMoveX = (calibratedGyroZ / 131.0) * MOUSE_SENSITIVITY_X * -1;
                mouseMoveY = (calibratedGyroX / 131.0) * MOUSE_SENSITIVITY_Y * -1;

                // Clamp movement
                mouseMoveX = constrain(mouseMoveX, -127, 127);
                mouseMoveY = constrain(mouseMoveY, -127, 127);

                // Deadzone
                if (abs(mouseMoveX) < 1)
                    mouseMoveX = 0;
                if (abs(mouseMoveY) < 1)
                    mouseMoveY = 0;

                bleMouse.move(mouseMoveX, mouseMoveY);

                // Touch controls for clicks with haptic feedback
                int buttonState = (touchRead(TOUCH_PIN_NUMBER) < TOUCH_THRESHOLD) ? LOW : HIGH;
                unsigned long currentTime = millis();

                if (buttonState == LOW && lastButtonState == HIGH)
                {
                    if (!isDragging)
                    {
                        clickCount++;
                        if (clickCount == 1)
                        {
                            lastClickTime = currentTime;
                            hapticFeedback(HAPTIC_CLICK);
                        }
                        firstClickTime = currentTime;
                    }
                }

                if (buttonState == HIGH && lastButtonState == LOW)
                {
                    if (isDragging)
                    {
                        bleMouse.release(MOUSE_LEFT);
                        isDragging = false;
                        hapticFeedback(HAPTIC_DRAG_END);
                        Serial.println("Drag Released");
                    }
                    firstClickTime = 0;
                }

                if (buttonState == LOW && !isDragging && (currentTime - firstClickTime >= LONG_PRESS_THRESHOLD))
                {
                    bleMouse.press(MOUSE_LEFT);
                    isDragging = true;
                    clickCount = 0;
                    hapticFeedback(HAPTIC_DRAG_START);
                    Serial.println("Drag Started");
                }

                if (clickCount > 0 && (currentTime - lastClickTime > DOUBLE_CLICK_TIMEOUT))
                {
                    if (clickCount == 1)
                    {
                        bleMouse.click(MOUSE_LEFT);
                        Serial.println("Left Click");
                    }
                    else if (clickCount == 2)
                    {
                        bleMouse.click(MOUSE_RIGHT);
                        hapticFeedback(HAPTIC_DOUBLE_CLICK);
                        Serial.println("Right Click (Double Tap)");
                    }
                    clickCount = 0;
                }

                lastButtonState = buttonState;
            }
            else
            {
                // NOT ORIENTED MODE - Gesture control
                if (isDragging)
                {
                    bleMouse.release(MOUSE_LEFT);
                    isDragging = false;
                    hapticFeedback(HAPTIC_DRAG_END);
                }

                // Check for new gestures
                if (newGestureAvailable)
                {
                    newGestureAvailable = false;

                    // Create local copy to avoid volatile issues
                    char localGesture[32];
                    strcpy(localGesture, (const char *)latestGesture);

                    // Process gesture commands with haptic feedback
                    if (strstr(localGesture, "circle") || strstr(localGesture, "Circle"))
                    {
                        // Circle gesture = Back button
                        bleMouse.click(MOUSE_BACK);
                        hapticFeedback(HAPTIC_GESTURE_OK);
                        Serial.println("Back button pressed (Circle gesture)");
                    }
                    else if (strstr(localGesture, "up") || strstr(localGesture, "down") ||
                             strstr(localGesture, "Up") || strstr(localGesture, "Down"))
                    {
                        // Up/Down gesture = Forward button
                        bleMouse.click(MOUSE_FORWARD);
                        hapticFeedback(HAPTIC_GESTURE_OK);
                        Serial.println("Forward button pressed (Up/Down gesture)");
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void readSensorData()
{
    while (i2cRead(0x3B, i2cData, 14))
        ;

    int16_t rawAccelX = ((i2cData[0] << 8) | i2cData[1]);
    int16_t rawAccelY = ((i2cData[2] << 8) | i2cData[3]);
    int16_t rawAccelZ = ((i2cData[4] << 8) | i2cData[5]);
    int16_t rawGyroX = ((i2cData[8] << 8) | i2cData[9]);
    int16_t rawGyroY = ((i2cData[10] << 8) | i2cData[11]);
    int16_t rawGyroZ = ((i2cData[12] << 8) | i2cData[13]);

    accelX = rawAccelZ;
    accelY = rawAccelX;
    accelZ = rawAccelY;
    gyroX_raw = rawGyroZ;
    gyroY_raw = rawGyroX;
    gyroZ_raw = rawGyroY;

    currentAccelX = (float)accelX - accelX_offset;
    currentAccelY = (float)accelY - accelY_offset;
    currentAccelZ = (float)accelZ;
    currentGyroX = (float)gyroX_raw - gyroX_offset;
    currentGyroY = (float)gyroY_raw - gyroY_offset;
    currentGyroZ = (float)gyroZ_raw - gyroZ_offset;
}

void loop()
{
    delay(5000);
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop)
{
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    Wire.write(data, length);
    return Wire.endTransmission(sendStop);
}

uint8_t i2cWrite2(uint8_t registerAddress, uint8_t data, bool sendStop)
{
    return i2cWrite(registerAddress, &data, 1, sendStop);
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes)
{
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    if (Wire.endTransmission(false))
        return 1;
    Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
    for (uint8_t i = 0; i < nbytes; i++)
    {
        if (Wire.available())
        {
            data[i] = Wire.read();
        }
        else
        {
            uint32_t timeOutTimer = micros();
            while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available())
                ;
            if (Wire.available())
            {
                data[i] = Wire.read();
            }
            else
            {
                return 2;
            }
        }
    }
    return 0;
}

void calibrateSensors()
{
    const int numReadings = 500; // Reduced from 1000 to save time and space
    long gyroX_sum = 0, gyroY_sum = 0, gyroZ_sum = 0;
    long accelX_sum = 0, accelY_sum = 0;

    for (int i = 0; i < numReadings; i++)
    {
        while (i2cRead(0x3B, i2cData, 14))
            ;

        int16_t rawAccelX = ((i2cData[0] << 8) | i2cData[1]);
        int16_t rawAccelY = ((i2cData[2] << 8) | i2cData[3]);
        int16_t rawAccelZ = ((i2cData[4] << 8) | i2cData[5]);
        int16_t rawGyroX = ((i2cData[8] << 8) | i2cData[9]);
        int16_t rawGyroY = ((i2cData[10] << 8) | i2cData[11]);
        int16_t rawGyroZ = ((i2cData[12] << 8) | i2cData[13]);

        accelX = rawAccelZ;
        accelY = rawAccelX;
        accelZ = rawAccelY;
        gyroX_raw = rawGyroZ;
        gyroY_raw = rawGyroX;
        gyroZ_raw = rawGyroY;

        gyroX_sum += gyroX_raw;
        gyroY_sum += gyroY_raw;
        gyroZ_sum += gyroZ_raw;
        accelX_sum += accelX;
        accelY_sum += accelY;

        delay(1);
    }

    gyroX_offset = (float)gyroX_sum / numReadings;
    gyroY_offset = (float)gyroY_sum / numReadings;
    gyroZ_offset = (float)gyroZ_sum / numReadings;
    accelX_offset = (float)accelX_sum / numReadings;
    accelY_offset = (float)accelY_sum / numReadings;
}
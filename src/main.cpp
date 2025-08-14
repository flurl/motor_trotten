#include "Adafruit_VL53L0X.h"
#include <Adafruit_SH1106.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>
#include <magic_enum.hpp>
#include <mutex>
#include <queue>

#define DEBUG 1

#if DEBUG
#define SerialDebug Serial
#else // DEBUG
// If DEBUG is not enabled, make SerialDebug a noop
#define SerialDebug \
    while (0)       \
    Serial
#endif

#define RESTART_DELAY 2000 // if a delayed restart is requested, this is the amount of ms after which the restart will happen

#define I2C_SDA_PIN 25 // SDA pin for I2C
#define I2C_SCL_PIN 26 // SCL pin for I2C

#define SCREEN_WIDTH 128 // OLED display width
#define SCREEN_HEIGHT 64 // OLED display height

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
#define INVALID_HEIGHT -10               // value indicating an invalid height
int16_t measuredHeight = INVALID_HEIGHT; // measured height in mm
#define HEIGHT_AVG_SIZE 5                // number of measurements to average
#define TABLE_HEIGHT 700                 // height of the table in mm when totally retracted measured from the ground up to the tableplate
#define MAX_HEIGHT 1200                  // maximum height in mm, used to limit the target height


Adafruit_SH1106 display(-1);

struct DisplayMessage {
    String text;              // Text to display
    uint8_t textSize = 1;     // Text size for display
    uint8_t x = 0;            // X position for display
    uint8_t y = 0;            // Y position for display
    uint16_t duration = 2000; // Duration in milliseconds to display the message
    unsigned long startTime;  // Start time of the message display
    bool isDisplayed = false; // Flag to indicate if the message is currently displayed
};
std::queue<DisplayMessage> displayMessageQueue;

// Just for referring, these are the touch sensor indices
#define TOUCH_1 0
#define TOUCH_2 1
#define TOUCH_3 2
#define TOUCH_4 3
#define TOUCH_PLUS 4
#define TOUCH_MINUS 5
#define NUM_TOUCH_SENSORS 6

// Lower threshold values mean higher sensitivity, but also a greater chance of false positives
#define TOUCH_DEFAULT_THRESHOLD_PERCENT 0.08                          // Default threshold for touch detection 0.0 - 1.0 (corresponds to 0% - 100%)
#define TOUCH_1_THRESHOLD_PERCENT 0.09                                // Threshold for touch sensor 1
#define TOUCH_2_THRESHOLD_PERCENT TOUCH_DEFAULT_THRESHOLD_PERCENT     // Threshold for touch sensor 2
#define TOUCH_3_THRESHOLD_PERCENT TOUCH_DEFAULT_THRESHOLD_PERCENT     // Threshold for touch sensor 3
#define TOUCH_4_THRESHOLD_PERCENT TOUCH_DEFAULT_THRESHOLD_PERCENT     // Threshold for touch sensor 4
#define TOUCH_PLUS_THRESHOLD_PERCENT TOUCH_DEFAULT_THRESHOLD_PERCENT  // Threshold for touch sensor 5 (plus)
#define TOUCH_MINUS_THRESHOLD_PERCENT TOUCH_DEFAULT_THRESHOLD_PERCENT // Threshold for touch sensor 6 (minus)

#define TOUCH_MIN_DURATION 200         // Minimum duration in milliseconds for a touch to be considered valid
#define TOUCH_LONG_PRESS_DURATION 1000 // Duration in milliseconds for long press detection
#define TOUCH_DEBOUNCE_DURATION 500    // Debounce duration in milliseconds for touch sensors
#define TOUCH_AVG_SIZE 3               // Number of readings to average for touch sensors
#define TOUCH_MEASURE_CYCLES 0x2000    // Measure cycles
#define TOUCH_SLEEP_CYCLES 0x2000      // Sleep between measurements

int touchPins[NUM_TOUCH_SENSORS] = {32, 33, 27, 14, 12, 13}; // Pin numbers for touch sensors

struct TouchSensor {
    float baseline = 0.0;                              // Baseline value for touchRead
    float threshold = TOUCH_DEFAULT_THRESHOLD_PERCENT; // Threshold for touch detection
    float average = 0.0;                               // average across last reading
    bool isTouched = false;                            // Flag to indicate if touch is detected
    bool pressDetected = false;                        // Flag to indicate if a press is detected
    unsigned long lastStartTouchTime = 0;              // Last time the sensor was touched
    bool longPressDetected = false;                    // Flag for long press detection
    bool isActive = false;                             // Flag to indicate if the sensor is active
    int pin;                                           // Pin number for the touch sensor
};

// create an array of touch sensors
TouchSensor touchSensors[NUM_TOUCH_SENSORS] = {};

#define SW_SETUP_PIN 16 // Pin for setup switch
bool setupSwitchPressed = false;

Preferences preferences;
int16_t heightOffset = -1;
uint16_t presets[4] = {0, 0, 0, 0}; // saved height presets

std::mutex stateMtx;
enum State {
    IDLE,
    SETUP,
    SET_HEIGHT,
    SAVE_PRESET,
    CALIBRATION,
    DELAYED_RESTART,
    ERROR
};
State state = IDLE;

enum ErrorCode {
    NO_ERROR,
    ERROR_INVALID_MEASUREMENT,
    ERROR_INVALID_HEIGHT,
    ERROR_ENDSTOP_TRIGGERED,
    ERROR_INVALID_DISTANCE,
};
ErrorCode errorCode = NO_ERROR;      // Current error code
int16_t targetHeight = TABLE_HEIGHT; // Target height for the action, measured from the ground up to the tableplate

#define MAX_DISTANCE_DELTA_PER_SECOND 10 // Maximum distance change per second in mm
int16_t lastHeight = INVALID_HEIGHT;     // Last measured height in mm


#define UPPER_ENDSTOP_PIN 5
#define LOWER_ENDSTOP_PIN 4
#define UPPER_ENDSTOP_ACTIVE HIGH // Active state for upper endstop
#define LOWER_ENDSTOP_ACTIVE LOW  // Active state for lower endstop
bool upperEndstopTriggered = false;
bool lowerEndstopTriggered = false;

#define PWM_R_PIN 23
#define PWM_L_PIN 22
#define DIRECTION_UP 0
#define DIRECTION_DOWN 1

String ssid;
String password;
AsyncWebServer server(80);
#define WIFI_PASSWORD "motortrotten"

// forward declarations
int16_t getHeight();
uint16_t getTableHeight();


void loadPreferences() {
    preferences.begin("settings", true);
    heightOffset = preferences.getUInt("heightOffset", -1);
    for (int i = 0; i < 4; i++) {
        presets[i] = preferences.getUInt(("preset" + String(i)).c_str(), 0);
    }
    targetHeight = preferences.getUInt("targetHeight", TABLE_HEIGHT);
    ssid = preferences.getString("ssid", "");
    password = preferences.getString("password", WIFI_PASSWORD);
    preferences.end();
}

void savePreferences() {
    preferences.begin("settings", false);
    preferences.putUInt("heightOffset", heightOffset);
    for (int i = 0; i < 4; i++) {
        preferences.putUInt(("preset" + String(i)).c_str(), presets[i]);
    }
    preferences.putUInt("targetHeight", targetHeight);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.end();
}

void enqueueDisplayMessage(const DisplayMessage &msg) {
    if (displayMessageQueue.size() >= 3) {
        displayMessageQueue.pop();
    }
    displayMessageQueue.push(msg);
}

void enqueueDisplayMessage(const String &text, uint8_t textSize = 1, uint16_t duration = 2000) {
    DisplayMessage msg;
    msg.text = text;
    msg.textSize = textSize;
    msg.duration = duration;
    enqueueDisplayMessage(msg);
}

bool dequeueDisplayMessage(DisplayMessage &msg) {
    if (!displayMessageQueue.empty()) {
        msg = displayMessageQueue.front();
        displayMessageQueue.pop();
        return true;
    }
    return false;
}

bool dequeueDisplayMessage() {
    if (!displayMessageQueue.empty()) {
        displayMessageQueue.pop();
        return true;
    }
    return false;
}

bool getDisplayMessage(DisplayMessage *&msg) {
    if (!displayMessageQueue.empty()) {
        msg = &displayMessageQueue.front();
        return true;
    }
    return false;
}

void displayNetworkInfo() {
    display.setTextSize(1);
    if (WiFi.getMode() == WIFI_STA && WiFi.isConnected()) {
        display.print("SSID: ");
        display.println(WiFi.SSID());
        display.print("IP: ");
        display.println(WiFi.localIP().toString());
    } else if (WiFi.getMode() == WIFI_AP) {
        display.print("SSID: ");
        display.println(WiFi.softAPSSID());
        display.print("PW: ");
        display.println(WIFI_PASSWORD);
        display.print("IP: ");
        display.println(WiFi.softAPIP().toString());
    } else {
        display.println("No WiFi");
    }
}


void displayHeightInfo() {
    display.setTextSize(1);
    display.print("Measured: ");
    if (getHeight() <= -10) {
        display.println("N/A");
    } else {
        display.print(getHeight() / 10);
        display.println(" cm");
    }
    display.print("Offset: ");
    if (heightOffset == -1) {
        display.println("N/A");
    } else {
        display.print(heightOffset / 10);
        display.println(" cm");
    }
}

void updateDisplay() {
    display.clearDisplay();

    DisplayMessage *msg;
    if (getDisplayMessage(msg)) {
        // SerialDebug.println("Displaying message: " + msg->text);
        // SerialDebug.println("Duration: " + String(msg->duration) + " ms");
        // SerialDebug.println("Start time: " + String(msg->startTime));
        // SerialDebug.println("Current time: " + String(millis()));
        // SerialDebug.println("Elapsed time: " + String(millis() - msg->startTime) + " ms");
        if (!msg->isDisplayed) {
            SerialDebug.println("Message not displayed yet, setting display state.");
            msg->isDisplayed = true;   // Mark the message as displayed
            msg->startTime = millis(); // Set the start time for the message
        }
        display.setCursor(msg->x, msg->y);
        display.setTextSize(msg->textSize);
        display.setTextColor(WHITE);
        display.println(msg->text);
        display.display();
        if (millis() - msg->startTime > msg->duration) {
            SerialDebug.println("Message duration exceeded, removing message: " + msg->text);
            dequeueDisplayMessage();
            return;
        }
        return;
    }

    display.setCursor(0, 0);
    switch (state) {
    case SETUP:
        display.setTextSize(2);
        display.println("SETUP");
        displayNetworkInfo();
        displayHeightInfo();
        break;
    case CALIBRATION:
        display.setTextSize(1);
        display.println("CALIBRATION");
        display.println("Wait for endstop\nreach. Then press\nsetup switch to set\nthe height offset.");
        break;
    case SET_HEIGHT:
    case SAVE_PRESET:
    case IDLE:
        display.setTextSize(2);
        display.print(String(getTableHeight() / 10) + "/" + String(targetHeight / 10) + "cm\n");
        if (setupSwitchPressed) {
            displayNetworkInfo();
        }
        displayHeightInfo();
        break;
    case ERROR:
        display.setTextSize(1);
        display.println("ERROR");
        display.print(magic_enum::enum_name(errorCode).data());
        break;
    case DELAYED_RESTART:
        break;
    }
    display.display();
}

void measureHeight() {
    static bool firstRun = true;
    VL53L0X_RangingMeasurementData_t measure;

    if (firstRun) {
        uint16_t tmpAvg = 0;
        firstRun = false;
        for (uint8_t i = 0; i < HEIGHT_AVG_SIZE * 2; i++) {
            lox.rangingTest(&measure, false);
            if (measure.RangeStatus != 4) { // phase failures have incorrect data
                tmpAvg += measure.RangeMilliMeter;
            } else {
                SerialDebug.println("Out of range, skipping measurement");
                i--; // Decrement i to retry this measurement
            }
            delay(100);
        }
        measuredHeight = tmpAvg / (HEIGHT_AVG_SIZE * 2); // Initialize with average of first measurements
    }

    lox.rangingTest(&measure, false);                                                                                            // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {                                                                                              // phase failures have incorrect data
        measuredHeight = measuredHeight * (1.0 - (1.0 / HEIGHT_AVG_SIZE)) + (measure.RangeMilliMeter * (1.0 / HEIGHT_AVG_SIZE)); // Simple moving average
        // SerialDebug.print("Measured Height (mm): ");
        // SerialDebug.println(measuredHeight);
    } else {
        // SerialDebug.println("Out of range");
        measuredHeight = INVALID_HEIGHT; // Reset to -1 if out of range
    }
}

// This will return the height considering the height offset of the sensor.
// Since we want to account for inaccurate measurements we define -10 as the value
// for no valid measurement.
// For example, the offset is 100mm, and the measured height is 90mm, we still
// treat it as a valid measurement and we return 0.
// But if the difference is more than -10mm we return the measured height minus the offset
// to indicate an error.
int16_t getHeight() {
    if (measuredHeight == INVALID_HEIGHT) {
        return INVALID_HEIGHT; // No valid measurement
    }

    // Something must be wrong if we get a negative height
    // accounting for the tolerqnce of -10mm
    if (measuredHeight - heightOffset < -10) {
        return INVALID_HEIGHT;
    }

    // A negative value, but bigger than -10 is still a valid measurement
    // so we return 0
    if (measuredHeight - heightOffset < 0) {
        return 0;
    }

    return measuredHeight - heightOffset;
}


// This function returns the real height of the table
// from the ground up to the tableplate
uint16_t getTableHeight() {
    return getHeight() + TABLE_HEIGHT;
}

void readTouchSensors() {
    static bool firstRun = true;
    for (int i = 0; i < NUM_TOUCH_SENSORS; i++) {
        if (firstRun) {
            uint32_t tmpAvg = 0;
            for (uint8_t n = 0; n < TOUCH_AVG_SIZE * 2; n++) {
                uint8_t reading = touchRead(touchPins[i]);
                tmpAvg += reading;
                // if (i == 1) {
                //     SerialDebug.print("init reading: ");
                //     SerialDebug.println(reading);
                // }
            }
            touchSensors[i].average = tmpAvg / (TOUCH_AVG_SIZE * 2); // Initialize with average of first measurements
            touchSensors[i].baseline = touchSensors[i].average;      // Set initial baseline to average
            SerialDebug.print("Touch sensor ");
            SerialDebug.print(i);
            SerialDebug.print(" initialized with baseline: ");
            SerialDebug.println(touchSensors[i].baseline);
        }

        int touchValue = touchRead(touchSensors[i].pin);
        // if (i == 1) {
        //     SerialDebug.print(" reading: ");
        //     SerialDebug.println(touchValue);
        //     SerialDebug.print("  Average: ");
        //     SerialDebug.println(touchSensors[i].average);
        //     SerialDebug.print("  Baseline: ");
        //     SerialDebug.println(touchSensors[i].baseline);
        // }
        touchSensors[i].average = (touchSensors[i].average * (1.0 - (1.0 / (TOUCH_AVG_SIZE)))) + (touchValue * (1.0 / (TOUCH_AVG_SIZE))); // Simple moving average
        touchSensors[i].isTouched = (touchSensors[i].average < (touchSensors[i].baseline * (1.0 - touchSensors[i].threshold)));
        // if (!touchSensors[i].isTouched) {
        //     if (millis() - touchSensors[i].lastStartTouchTime > TOUCH_MIN_DURATION && !touchSensors[i].longPressDetected) {
        //         touchSensors[i].pressDetected = true; // Press detected
        //     }
        //     touchSensors[i].lastStartTouchTime = 0;
        //     touchSensors[i].longPressDetected = false;                                      // Reset long press flag
        //     touchSensors[i].baseline = touchSensors[i].baseline * 0.99 + touchValue * 0.01; // Slowly adjust baseline
        // } else {
        //     if (touchSensors[i].lastStartTouchTime == 0) {
        //         touchSensors[i].lastStartTouchTime = millis(); // Record the time when touched
        //     } else if (millis() - touchSensors[i].lastStartTouchTime <= TOUCH_DEBOUNCE_DURATION) {
        //         // If the touch is still detected within the debounce duration, ignore it
        //         touchSensors[i].pressDetected = false; // Reset press detected to avoid multiple triggers
        //         continue;
        //     } else if (millis() - touchSensors[i].lastStartTouchTime > TOUCH_LONG_PRESS_DURATION) {
        //         touchSensors[i].longPressDetected = true; // Long press detected
        //     }
        //     SerialDebug.print("Touch sensor ");
        //     SerialDebug.println(i);
        //     SerialDebug.print("  Value: ");
        //     SerialDebug.println(touchValue);
        //     SerialDebug.print("  Average: ");
        //     SerialDebug.println(touchSensors[i].average);
        //     SerialDebug.print("  Baseline: ");
        //     SerialDebug.println(touchSensors[i].baseline);
        // }
        if (touchSensors[i].isTouched) {
            if (!touchSensors[i].isActive && (millis() - touchSensors[i].lastStartTouchTime > TOUCH_DEBOUNCE_DURATION)) {
                touchSensors[i].isActive = true;               // Mark the sensor as active
                touchSensors[i].lastStartTouchTime = millis(); // Record the time when touched
            }
            // If the touch sensor has been touched continuously for longer than the long press duration
            // it's interpreted as a long press
            if (millis() - touchSensors[i].lastStartTouchTime > TOUCH_LONG_PRESS_DURATION) {
                touchSensors[i].longPressDetected = true; // Long press detected
            }
        } else {
            touchSensors[i].baseline = touchSensors[i].baseline * 0.99 + touchValue * 0.01; // Slowly adjust baseline
            if (touchSensors[i].isActive && millis() - touchSensors[i].lastStartTouchTime > TOUCH_MIN_DURATION && !touchSensors[i].longPressDetected) {
                touchSensors[i].pressDetected = true; // Press detected
            } else {
                touchSensors[i].pressDetected = false;
            }
            touchSensors[i].isActive = false;          // Mark the sensor as inactive
            touchSensors[i].longPressDetected = false; // Reset long press flag
        }
    }
    if (firstRun) {
        firstRun = false; // Set firstRun to false after the first initialization
    }
}

void checkInputs() {
    if (digitalRead(UPPER_ENDSTOP_PIN) == UPPER_ENDSTOP_ACTIVE) {
        upperEndstopTriggered = true;
        // SerialDebug.println("Upper endstop triggered");
    } else {
        upperEndstopTriggered = false;
    }

    if (digitalRead(LOWER_ENDSTOP_PIN) == LOWER_ENDSTOP_ACTIVE) {
        lowerEndstopTriggered = true;
        // SerialDebug.println("Lower endstop triggered");
    } else {
        lowerEndstopTriggered = false;
    }

    if (digitalRead(SW_SETUP_PIN) == LOW) {
        setupSwitchPressed = true;
        SerialDebug.println("Setup switch pressed");
    } else {
        setupSwitchPressed = false;
    }
}

void stopMotor() {
    // SerialDebug.println("Stopping motor");
    digitalWrite(PWM_L_PIN, LOW);
    digitalWrite(PWM_R_PIN, LOW);
}

void activateMotor(uint8_t direction) {
    if (direction == DIRECTION_UP) {
        if (upperEndstopTriggered) {
            // SerialDebug.println("Upper endstop triggered, stopping motor");
            stopMotor();
            return;
        }
        // SerialDebug.println("Activating motor in UP direction");
        digitalWrite(PWM_L_PIN, LOW);
        digitalWrite(PWM_R_PIN, HIGH);
    } else if (direction == DIRECTION_DOWN) {
        if (lowerEndstopTriggered) {
            // SerialDebug.println("Lower endstop triggered, stopping motor");
            stopMotor();
            return;
        }
        // SerialDebug.println("Activating motor in DOWN direction");
        digitalWrite(PWM_R_PIN, LOW);
        digitalWrite(PWM_L_PIN, HIGH);
    }
}

void savePreset(int index, int16_t value) {
    presets[index] = value;
    preferences.begin("settings", false);
    preferences.putUInt(("preset" + String(index)).c_str(), value);
    preferences.end();
    enqueueDisplayMessage("Preset " + String(index) + " saved:\n" + String(value / 10) + " cm");
    SerialDebug.print("Preset ");
    SerialDebug.print(index);
    SerialDebug.println(" saved.");
}

void setTargetHeight(int16_t height) {
    if (height < TABLE_HEIGHT || height > MAX_HEIGHT) {
        SerialDebug.println("Invalid height, must be between " + String(TABLE_HEIGHT) + " and " + String(MAX_HEIGHT));
        state = IDLE; // Return to IDLE state if height is invalid
        if (height < TABLE_HEIGHT) {
            enqueueDisplayMessage("Min height\n" + String(TABLE_HEIGHT / 10) + "cm", 2);
        } else {
            enqueueDisplayMessage("Max height\n" + String(MAX_HEIGHT / 10) + "cm", 2);
        }
        return;
    }
    targetHeight = height;
    SerialDebug.print("Target height set to: ");
    SerialDebug.println(targetHeight);
    savePreferences(); // Save the target height to preferences
}

void scheduleAction() {
    // in case a restart is scheduled, we do not want to do interrupt it
    if (state == DELAYED_RESTART) {
        return;
    }
    if (touchSensors[TOUCH_1].longPressDetected) {
        // SerialDebug.println("Touch 1 long press detected, saving preset...");
        state = SAVE_PRESET;
    } else if (touchSensors[TOUCH_1].pressDetected) {
        // SerialDebug.println("Touch 1 detected, executing action...");
        setTargetHeight(presets[TOUCH_1]);
        state = SET_HEIGHT;
        // the press has been handled, reset the flag
        touchSensors[TOUCH_1].pressDetected = false;
    } else if (touchSensors[TOUCH_2].longPressDetected) {
        // SerialDebug.println("Touch 2 long press detected, saving preset...");
        state = SAVE_PRESET;
    } else if (touchSensors[TOUCH_2].pressDetected) {
        // SerialDebug.println("Touch 2 detected, executing action...");
        setTargetHeight(presets[TOUCH_2]);
        state = SET_HEIGHT;
        // the press has been handled, reset the flag
        touchSensors[TOUCH_2].pressDetected = false;
    } else if (touchSensors[TOUCH_3].longPressDetected) {
        // SerialDebug.println("Touch 3 long press detected, saving preset...");
        state = SAVE_PRESET;
    } else if (touchSensors[TOUCH_3].pressDetected) {
        // SerialDebug.println("Touch 3 detected, executing action...");
        setTargetHeight(presets[TOUCH_3]);
        state = SET_HEIGHT;
        // the press has been handled, reset the flag
        touchSensors[TOUCH_3].pressDetected = false;
    } else if (touchSensors[TOUCH_4].longPressDetected) {
        // SerialDebug.println("Touch 4 long press detected, saving preset...");
        state = SAVE_PRESET;
    } else if (touchSensors[TOUCH_4].pressDetected) {
        // SerialDebug.println("Touch 4 detected, executing action...");
        setTargetHeight(presets[TOUCH_4]);
        state = SET_HEIGHT;
        // the press has been handled, reset the flag
        touchSensors[TOUCH_4].pressDetected = false;
    } else if (touchSensors[TOUCH_PLUS].pressDetected || touchSensors[TOUCH_PLUS].longPressDetected) {
        // SerialDebug.println("Touch 5 detected, executing action...");
        if (!upperEndstopTriggered) {
            setTargetHeight(targetHeight + 10); // Increase target height by 10mm
            state = SET_HEIGHT;
        }
        // the press has been handled, reset the flag
        touchSensors[TOUCH_PLUS].pressDetected = false;
    } else if (touchSensors[TOUCH_MINUS].pressDetected || touchSensors[TOUCH_MINUS].longPressDetected) {
        // SerialDebug.println("Touch 6 detected, executing action...");
        if (!lowerEndstopTriggered) {
            if (targetHeight >= 10) {
                setTargetHeight(targetHeight - 10); // Decrease target height by 10mm
            } else {
                setTargetHeight(0); // Prevent negative height
            }
            state = SET_HEIGHT;
        }
        // the press has been handled, reset the flag
        touchSensors[TOUCH_MINUS].pressDetected = false;
    }
}

void executeAction() {
    static unsigned long restartScheduledAt = 0; // the time when the restart was scheduled. The restart will happen at restartScheduledAt + PERIOD
    static bool restartScheduled = false;


    switch (state) {
    case DELAYED_RESTART:
        stopMotor();
        if (!restartScheduled) {
            restartScheduled = true;
            restartScheduledAt = millis();
        }
        if (millis() - restartScheduledAt > RESTART_DELAY) {
            SerialDebug.println("Restarting...");
            ESP.restart();
        }
        break;
    case SETUP:
        SerialDebug.println("Setup mode, please set height offset.");
        if (setupSwitchPressed) {
            delay(1000); // Wait for 1 second to give the user time to release the switch
            targetHeight = 0;
            if (!lowerEndstopTriggered) {
                activateMotor(DIRECTION_DOWN);
            }
            state = CALIBRATION;
        }
        break;

    case CALIBRATION:
        SerialDebug.println("Calibration mode, measuring height...");
        if (lowerEndstopTriggered) {
            stopMotor();
        }
        if (setupSwitchPressed) {
            heightOffset = getHeight();
            targetHeight = TABLE_HEIGHT;
            SerialDebug.print("Height offset set to ");
            SerialDebug.println(heightOffset);
            enqueueDisplayMessage("Height offset\nset to " + String(heightOffset / 10) + " cm\nRestarting...", 1);
            savePreferences(); // Save the height offset to preferences
            SerialDebug.println("Restarting...");
            state = DELAYED_RESTART;
        }
        break;

    case SET_HEIGHT:
        // SerialDebug.print("Setting height to: ");
        // SerialDebug.println(targetHeight);
        if (measuredHeight == INVALID_HEIGHT) {
            SerialDebug.println("No valid measurement available, cannot set height.");
            return; // Exit if no valid measurement
        }
        if (getTableHeight() < targetHeight) {
            activateMotor(DIRECTION_UP);
        } else if (getTableHeight() > targetHeight) {
            activateMotor(DIRECTION_DOWN);
        } else {
            stopMotor();  // Stop motor if target height is reached
            state = IDLE; // Return to IDLE state after setting height
        }
        break;

    case SAVE_PRESET:
        SerialDebug.print("Saving preset: ");
        SerialDebug.println(targetHeight);
        for (int i = 0; i < 4; i++) {
            if (touchSensors[i].longPressDetected) { // Find first sensor with long press
                savePreset(i, targetHeight);
                touchSensors[i].longPressDetected = false;     // Reset long press flag
                touchSensors[i].lastStartTouchTime = millis(); // Reset last touch time, this prevents immediate re-triggering
                break;
            }
        }
        state = IDLE; // Go back to IDLE, the message queue will handle showing the confirmation.
        break;

    case IDLE:
        // Remain in IDLE state
        break;

    default:
        stopMotor(); // Stop motor if in undefined state
        break;
    }
}

// Do some sanity checks to ensure the system is in a valid state
// before executing actions or setting target heights
// Returns true if the system state is valid, false otherwise
bool isSystemStateValid() {
    if ((targetHeight < TABLE_HEIGHT || targetHeight > MAX_HEIGHT) && state != CALIBRATION) {
        SerialDebug.println("Invalid target height.");
        SerialDebug.print("Target Height: ");
        SerialDebug.println(targetHeight);
        state = ERROR;                    // Set state to ERROR if target height is invalid
        errorCode = ERROR_INVALID_HEIGHT; // Set error code for invalid height
        return false;
    }
    if (getHeight() == INVALID_HEIGHT) {
        SerialDebug.println("No valid height measurement available.");
        SerialDebug.print("Measured Height: ");
        SerialDebug.println(measuredHeight);
        SerialDebug.print("Height accounting for offset: ");
        SerialDebug.println(getHeight());
        state = ERROR;                         // Set state to ERROR if no valid measurement is available
        errorCode = ERROR_INVALID_MEASUREMENT; // Set error code for no valid measurement
        return false;
    }
    if (lastHeight == INVALID_HEIGHT) {
        lastHeight = getHeight(); // Initialize lastHeight if it is -1
    }
    // Check if the height change is within the allowed delta
    if (abs(getHeight() - lastHeight) > MAX_DISTANCE_DELTA_PER_SECOND) {
        SerialDebug.println("Height change exceeds maximum allowed delta.");
        SerialDebug.print("Current height: ");
        SerialDebug.println(getHeight());
        SerialDebug.print("Last height: ");
        SerialDebug.println(lastHeight);
        state = ERROR;                      // Set state to ERROR if height change exceeds maximum allowed delta
        errorCode = ERROR_INVALID_DISTANCE; // Set error code for invalid distance
        return false;
    }
    lastHeight = getHeight(); // Update lastHeight to the current measured height

    // Check if the endstop for the direction of movement is triggered
    if (upperEndstopTriggered && targetHeight > getTableHeight() && state == SET_HEIGHT) {
        SerialDebug.println("Upper endstop triggered, cannot move up.");
        state = ERROR;                       // Set state to ERROR if upper endstop is triggered
        errorCode = ERROR_ENDSTOP_TRIGGERED; // Set error code for upper endstop triggered
        return false;
    }
    // In calibration mode we do not trigger an error for the lower endstop
    // because we want to move down to the lower endstop to set the height offset
    if (lowerEndstopTriggered && targetHeight < getTableHeight() && state == SET_HEIGHT) {
        SerialDebug.println("Lower endstop triggered, cannot move down.");
        SerialDebug.print("Current height: ");
        SerialDebug.println(getTableHeight());
        SerialDebug.print("Target height: ");
        SerialDebug.println(targetHeight);
        SerialDebug.print("Measured height: ");
        SerialDebug.println(getHeight());
        SerialDebug.print("Height offset: ");
        SerialDebug.println(heightOffset);
        state = ERROR;                       // Set state to ERROR if lower endstop is triggered
        errorCode = ERROR_ENDSTOP_TRIGGERED; // Set error code for lower endstop triggered
        return false;
    }


    return true;
}

void createAP() {
    WiFi.mode(WIFI_AP);
    // Generate unique AP name using chip ID
    uint64_t chipid = ESP.getEfuseMac();
    String apName = "MoTrot-" + String((uint32_t)(chipid & 0xFFFFFFF), HEX);
    WiFi.softAP(apName.c_str(), WIFI_PASSWORD);

    SerialDebug.print("AP started with SSID: ");
    SerialDebug.println(apName);
    SerialDebug.print("AP IP address: ");
    SerialDebug.println(WiFi.softAPIP());
}


void setup() {
    Serial.begin(115200);

    // wait until serial port opens for native USB devices
    while (!Serial) {
        delay(1);
    }

    // Load preferences
    loadPreferences();

    // initialize pins
    pinMode(UPPER_ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(LOWER_ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(PWM_R_PIN, OUTPUT);
    pinMode(PWM_L_PIN, OUTPUT);
    pinMode(SW_SETUP_PIN, INPUT_PULLUP);

    // Initialize I2C with custom pins for ESP32
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // SDA = 25, SCL = 26

    // initialize the display
    SerialDebug.println("Initializing display...");
    display.begin(SH1106_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();
    SerialDebug.println("Display initialized.");
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("MotorTrotten");
    display.println("Initializing...");
    display.display();

    SerialDebug.println("Adafruit VL53L0X test");
    if (!lox.begin()) {
        SerialDebug.println(F("Failed to boot VL53L0X"));
        while (1)
            ;
    }
    SerialDebug.println("initializing height measurement...");
    measureHeight(); // Measure height to initialize measuredHeight
    SerialDebug.println("Height measurement initialized.");

    // Try to connect to WiFi
    if (ssid.length() > 0 && password.length() > 0) {
        SerialDebug.print("Connecting to WiFi SSID: ");
        SerialDebug.println(ssid);
        SerialDebug.print("Password: ");
        SerialDebug.println(password);
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid.c_str(), password.c_str());

        unsigned long startAttemptTime = millis();
        const unsigned long wifiTimeout = 10000; // 10 seconds timeout

        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
            delay(500);
            SerialDebug.print(".");
        }
        SerialDebug.println();

        if (WiFi.status() == WL_CONNECTED) {
            SerialDebug.print("Connected! IP address: ");
            SerialDebug.println(WiFi.localIP());
        } else {
            SerialDebug.println("WiFi connection failed, starting fallback AP...");
            WiFi.disconnect(true);
            createAP();
        }
    } else {
        SerialDebug.println("No WiFi credentials found, starting fallback AP...");
        createAP();
    }
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = "<!DOCTYPE html><html><head><title>MotorTrotten Setup</title>";
        html += "<style>";
        html += "body { font-family: Arial, sans-serif; background: #f4f4f4; margin: 0; padding: 20px; }";
        html += "h2 { color: #333; }";
        html += "form { background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); max-width: 400px; margin: auto; }";
        html += "label { display: block; margin-bottom: 12px; color: #444; }";
        html += "input[type='number'], input[type='text'], input[type='password'] { width: 100%; padding: 8px; margin-top: 4px; border: 1px solid #ccc; border-radius: 4px; }";
        html += "input[type='submit'] { background: #007bff; color: #fff; border: none; padding: 10px 20px; border-radius: 4px; cursor: pointer; margin-top: 10px; }";
        html += "input[type='submit']:hover { background: #0056b3; }";
        html += "</style></head><body>";
        html += "<h2>MotorTrotten Configuration</h2>";
        html += "<form method='POST' action='/save'>";
        html += "<label>Preset 1 (mm): <input type='number' name='preset0' value='" + String(presets[0]) + "'></label>";
        html += "<label>Preset 2 (mm): <input type='number' name='preset1' value='" + String(presets[1]) + "'></label>";
        html += "<label>Preset 3 (mm): <input type='number' name='preset2' value='" + String(presets[2]) + "'></label>";
        html += "<label>Preset 4 (mm): <input type='number' name='preset3' value='" + String(presets[3]) + "'></label>";
        html += "<label>Height Offset (mm): <input type='number' name='heightOffset' value='" + String(heightOffset) + "'></label>";
        html += "<label>WiFi SSID: <input type='text' name='ssid' value='" + ssid + "'></label>";
        html += "<label>WiFi Password: <input type='password' name='password' value='" + password + "'></label>";
        html += "<input type='submit' value='Save'>";
        html += "</form></body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
        // lock the state mtx so that when the next time in loop() the state is evaluated
        // the saved preferences are already taken in account
        std::lock_guard<std::mutex> lck(stateMtx);
        if (request->hasParam("preset0", true))
            presets[0] = request->getParam("preset0", true)->value().toInt();
        if (request->hasParam("preset1", true))
            presets[1] = request->getParam("preset1", true)->value().toInt();
        if (request->hasParam("preset2", true))
            presets[2] = request->getParam("preset2", true)->value().toInt();
        if (request->hasParam("preset3", true))
            presets[3] = request->getParam("preset3", true)->value().toInt();
        if (request->hasParam("heightOffset", true))
            heightOffset = request->getParam("heightOffset", true)->value().toInt();
        if (request->hasParam("ssid", true))
            ssid = request->getParam("ssid", true)->value();
        if (request->hasParam("password", true))
            password = request->getParam("password", true)->value();
        savePreferences();
        enqueueDisplayMessage("Configuration saved\nRestarting...", 1);
        state = DELAYED_RESTART;
        request->redirect("/");
    });
    server.begin();

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else { // U_SPIFFS
                type = "filesystem";
            }

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            SerialDebug.println("Start updating " + type);
        })
        .onEnd([]() {
            SerialDebug.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            SerialDebug.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            SerialDebug.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                SerialDebug.println("Auth Failed");
            } else if (error == OTA_BEGIN_ERROR) {
                SerialDebug.println("Begin Failed");
            } else if (error == OTA_CONNECT_ERROR) {
                SerialDebug.println("Connect Failed");
            } else if (error == OTA_RECEIVE_ERROR) {
                SerialDebug.println("Receive Failed");
            } else if (error == OTA_END_ERROR) {
                SerialDebug.println("End Failed");
            }
        });

    ArduinoOTA.begin();

    SerialDebug.println("Initializing touch sensors...");
    for (int i = 0; i < NUM_TOUCH_SENSORS; i++) {
        touchSensors[i].pin = touchPins[i];
        pinMode(touchSensors[i].pin, INPUT_PULLDOWN);
    }
    touchSensors[TOUCH_1].threshold = TOUCH_1_THRESHOLD_PERCENT;
    touchSensors[TOUCH_2].threshold = TOUCH_2_THRESHOLD_PERCENT;
    touchSensors[TOUCH_3].threshold = TOUCH_3_THRESHOLD_PERCENT;
    touchSensors[TOUCH_4].threshold = TOUCH_4_THRESHOLD_PERCENT;
    touchSensors[TOUCH_PLUS].threshold = TOUCH_PLUS_THRESHOLD_PERCENT;
    touchSensors[TOUCH_MINUS].threshold = TOUCH_MINUS_THRESHOLD_PERCENT;
    touchSetCycles(TOUCH_MEASURE_CYCLES, TOUCH_SLEEP_CYCLES);
    readTouchSensors(); // Read touch sensors to initialize their values
    SerialDebug.println("Touch sensors initialized.");
    enqueueDisplayMessage("Ready", 2);
    updateDisplay();
    if (digitalRead(SW_SETUP_PIN) == LOW) {
        state = SETUP;
        delay(2000); // give the user some time to release the button
    } else {
        state = IDLE;
    }
    SerialDebug.println("Setup complete.");
}

void loop() {
    measureHeight();
    readTouchSensors();
    checkInputs();
    scheduleAction();

    // ensure that state does not change in the middle of the iteration
    std::unique_lock<std::mutex> lck(stateMtx);

    // if no height offset is set, go into setup mode
    if (!(state == DELAYED_RESTART || state == ERROR || state == CALIBRATION) && heightOffset == -1) {
        SerialDebug.println("No height offset set, going into setup mode.");
        state = SETUP;
    }
    // don't check for errors if a restart is scheduled or we are already in ERROR state
    if (!(state == DELAYED_RESTART || state == ERROR) && !isSystemStateValid()) {
        SerialDebug.println("Error detected!");
        SerialDebug.println("State: " + String(state));
        SerialDebug.println("Error code: " + String(errorCode));
        SerialDebug.println("");
        state = ERROR;
        // magic_enum is used to get the enum name as a string
        enqueueDisplayMessage(String("Error occurred\n") + magic_enum::enum_name(errorCode).data(), 1);
    }
    updateDisplay();
    executeAction();

    lck.unlock();

    ArduinoOTA.handle();
}

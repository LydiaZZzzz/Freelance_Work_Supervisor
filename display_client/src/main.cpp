#include <WiFi.h>
#include "time.h"
#include <Wire.h>
#include "DFRobot_LcdDisplay.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

const char* ssid = "UW MPSK";
const char* password = "q>.C3MM!Pa";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -28800;  
const int   daylightOffset_sec = 3600;  

#define LED_PIN 7
#define SDA_PIN 8
#define SCL_PIN 9
#define BUTTON_PIN 6  
#define COIL_A1 1
#define COIL_A2 2
#define COIL_B1 3
#define COIL_B2 4

const int stepSequence[4][4] = {
    {1, 0, 1, 0}, 
    {0, 1, 1, 0}, 
    {0, 1, 0, 1}, 
    {1, 0, 0, 1}  
};

DFRobot_Lcd_IIC lcd(&Wire, 0x2C);
uint8_t timeLabelId;
uint8_t dateLabelId;
bool gifDisplayed = false;
int buttonPressCount = 0;  
unsigned long lastDebounceTime = 0;  
const int debounceDelay = 200;  

static BLEUUID serviceUUID("705eac08-e46d-4df9-a1d9-7e87a7597e2d");
static BLEUUID charUUID("9b85c59c-4170-4176-902c-8f2ee4a294e7");
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

uint8_t messageLabelId = 0;  

static void notifyCallback(
    BLERemoteCharacteristic* pBLERemoteCharacteristic,
    uint8_t* pData,
    size_t length,
    bool isNotify) {

    std::string receivedStr(reinterpret_cast<char*>(pData), length);
    Serial.print("Received data: ");
    Serial.println(receivedStr.c_str());

    if (messageLabelId != 0) {
        lcd.deleteString(messageLabelId);  
    }

    messageLabelId = lcd.drawString(10, 10, receivedStr.c_str(), 1, 0x800080);

    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        displayTimeAndDate(timeinfo);  
    }

    if (!gifDisplayed) {
        lcd.drawGif(200, 120, "pp.gif", 512);  
        gifDisplayed = true;  
    }
}

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.println("Connected to BLE Server!");
    }

    void onDisconnect(BLEClient* pclient) {
        connected = false;
        Serial.println("Disconnected from BLE Server!");
    }
};

bool connectToServer() {
    Serial.print("Connecting to BLE Server: ");
    Serial.println(myDevice->getAddress().toString().c_str());

    BLEClient* pClient = BLEDevice::createClient();
    Serial.println(" - Created BLE Client");

    pClient->setClientCallbacks(new MyClientCallback());
    pClient->connect(myDevice);
    Serial.println(" - Connected to BLE Server");

    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
        Serial.print("Service UUID not found: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Service found");

    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
        Serial.print("Characteristic UUID not found: ");
        Serial.println(charUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Characteristic found");

    if (pRemoteCharacteristic->canNotify()) {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }

    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = true;
            Serial.println("Found target BLE Server, stopping scan...");
        }
    }
};

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    lcd.begin();
    lcd.cleanScreen();  
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);  
    pinMode(COIL_A1, OUTPUT);
    pinMode(COIL_A2, OUTPUT);
    pinMode(COIL_B1, OUTPUT);
    pinMode(COIL_B2, OUTPUT);

    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(300);
        digitalWrite(LED_PIN, LOW);
        delay(300);
    }
    digitalWrite(LED_PIN, LOW);  
    Serial.println("Device startup complete!");

    Serial.println("Resetting stepper motor...");
    resetMotor();  
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("Fetching time...");

    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to fetch time, retrying...");
        delay(1000);
    }

    displayTimeAndDate(timeinfo);

    displayGIF();

    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);  
    Serial.println("Scanning for BLE Server...");
}

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastStepTime = 0;
    const int timeInterval = 1000;  
    const int stepInterval = 3000;  

    if (millis() - lastUpdate >= timeInterval) {
        lastUpdate = millis();
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            displayTimeAndDate(timeinfo);  
        }
    }

    if (millis() - lastStepTime >= stepInterval) {
        lastStepTime = millis();
        stepMotor(5, false, 30);  
    }

    checkButtonPress();  

    if (doConnect) {
        if (connectToServer()) {
            Serial.println("Successfully connected to BLE Server!");
        } else {
            Serial.println("Connection failed, rescanning...");
        }
        doConnect = false;
    }

    if (!connected && doScan) {
        BLEDevice::getScan()->start(0);
    }

    delay(100);
}

void displayTimeAndDate(struct tm timeinfo) {
    char timeStr[20];
    sprintf(timeStr, "Time: %02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    
    char dateStr[20];
    sprintf(dateStr, "Date: %04d-%02d-%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);

    if (timeLabelId == 0) {
        timeLabelId = lcd.drawString(10, 90, timeStr, 1, 0x800080);
        dateLabelId = lcd.drawString(10, 140, dateStr, 0, BLUE);
    } else {
        lcd.updateString(timeLabelId, 10, 90, timeStr, 1, 0x800080);
        lcd.updateString(dateLabelId, 10, 140, dateStr, 0, BLUE);
    }
}

void displayGIF() {
    if (!gifDisplayed) {
        lcd.drawGif(200, 120, "pp.gif", 512);  
        gifDisplayed = true;  
    }
}

void stepMotor(int stepDelay, bool reverse, int steps) {
    for (int step = 0; step < steps; step++) {  
        for (int i = 0; i < 4; i++) {
            int s = reverse ? (3 - i) : i; 
            digitalWrite(COIL_A1, stepSequence[s][0]);
            digitalWrite(COIL_A2, stepSequence[s][1]);
            digitalWrite(COIL_B1, stepSequence[s][2]);
            digitalWrite(COIL_B2, stepSequence[s][3]);
            delay(stepDelay);
        }
    }
}

void checkButtonPress() {
    if (digitalRead(BUTTON_PIN) == LOW) {  
        if (millis() - lastDebounceTime > debounceDelay) {  
            buttonPressCount++;  
            Serial.print("Button press count: ");
            Serial.println(buttonPressCount);

            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);

            if (buttonPressCount >= 5) {  
                Serial.println("Resetting stepper motor...");
                resetMotor();  
                buttonPressCount = 0;  
            }
        }
        lastDebounceTime = millis();  
        while (digitalRead(BUTTON_PIN) == LOW); 
    }
}

void resetMotor() {
    stepMotor(5, true, 200);  
    Serial.println("Stepper motor reset complete!");
}
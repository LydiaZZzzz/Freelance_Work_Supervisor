#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <drawingo_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

// BLE definitions
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID        "705eac08-e46d-4df9-a1d9-7e87a7597e2d"
#define CHARACTERISTIC_UUID "9b85c59c-4170-4176-902c-8f2ee4a294e7"

// Camera pin definitions
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

#define LED_GPIO_NUM      7  // LED pin

// Camera-related definitions
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

// Camera configuration
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422, GRAYSCALE, RGB565, JPEG
    .frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, // 0-63 lower number means higher quality
    .fb_count = 1,      // if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// BLE callback class
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Client connected successfully!");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Client disconnected, waiting for reconnection...");
        pServer->startAdvertising();
    }
};

// Initialize BLE
void setupBLE() {
    BLEDevice::init("BLE_Server");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE Server ready, waiting for Client connection...");
}

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // Initialize Serial
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // Initialize LED pin
    pinMode(LED_GPIO_NUM, OUTPUT);

    // Blink LED 5 times on startup
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_GPIO_NUM, HIGH);  // Turn on LED
        delay(300);                       // Delay 300ms
        digitalWrite(LED_GPIO_NUM, LOW);  // Turn off LED
        delay(300);                       // Delay 300ms
    }

    // Initialize BLE
    setupBLE();

    // Initialize camera
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continuous inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
 * @brief      Get data and run inferencing
 */
void loop()
{
    static unsigned long lastDetectionTime = 0; // Last detection time
    static unsigned long lastUploadTime = 0;   // Last upload time
    static unsigned long lastDrawingDetectedTime = 0; // Last time drawing was detected
    static bool hasDrawingDetected = false;    // Whether drawing was detected in the last 30 seconds

    const unsigned long detectionInterval = 2000;  // Detect every 5 seconds
    const unsigned long uploadInterval = 30000;    // Upload every 30 seconds
    const unsigned long noDrawingTimeout = 60000;  // 1-minute timeout for no drawing detection

    // Perform detection every 5 seconds
    if (millis() - lastDetectionTime >= detectionInterval) {
        lastDetectionTime = millis();

        // Allocate image buffer
        snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

        // Check if allocation was successful
        if(snapshot_buf == nullptr) {
            ei_printf("ERR: Failed to allocate snapshot buffer!\n");
            return;
        }

        // Capture image
        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_get_data;

        if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
            ei_printf("Failed to capture image\r\n");
            free(snapshot_buf);
            return;
        }

        // Run classifier
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }

        // Print inference results
        Serial.println("Current inference results:");
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            Serial.printf("  %s: %.2f%%\n", ei_classifier_inferencing_categories[i], result.classification[i].value * 100);
        }

        // Check if drawing is detected
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) {
                continue;
            }

            // If "drawing" is detected with confidence > 0.5
            if (strcmp(bb.label, "drawing") == 0 && bb.value > 0.5) {
                hasDrawingDetected = true;
                lastDrawingDetectedTime = millis();  // Update last drawing detection time
                Serial.printf("Drawing detected, confidence: %.2f%%\n", bb.value * 100);
                break;
            }
        }

        // Free image buffer
        free(snapshot_buf);
    }

    // Upload results every 30 seconds
    if (millis() - lastUploadTime >= uploadInterval) {
        lastUploadTime = millis();

        // Send BLE message based on detection result
        if (hasDrawingDetected) {
            if (deviceConnected) {
                String message = "drawing";
                pCharacteristic->setValue(message.c_str());
                pCharacteristic->notify();
                Serial.println("Sending data: drawing");
            }
        } else {
            if (deviceConnected) {
                String message = "Get back to work";
                pCharacteristic->setValue(message.c_str());
                pCharacteristic->notify();
                Serial.println("Sending data: Get back to work");
            }
        }

        // Reset detection status
        hasDrawingDetected = false;
    }

    // Turn on red LED if no drawing is detected for 1 minute
    if (millis() - lastDrawingDetectedTime >= noDrawingTimeout) {
        digitalWrite(LED_GPIO_NUM, HIGH);  // Turn on red LED
    } else {
        digitalWrite(LED_GPIO_NUM, LOW);   // Turn off red LED
    }

    // BLE connection management
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        pServer->startAdvertising();
        Serial.println("Starting new BLE advertising...");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
}

/**
 * @brief   Setup image sensor & start streaming
 */
bool ei_camera_init(void) {
    if (is_initialised) return true;

    // Initialize camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1); // flip it back
        s->set_brightness(s, 1); // up the brightness just a bit
        s->set_saturation(s, 0); // lower the saturation
    }

    is_initialised = true;
    return true;
}

/**
 * @brief      Capture, rescale and crop image
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if(!converted){
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

/**
 * @brief      Get data from camera
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }

    return 0;
}
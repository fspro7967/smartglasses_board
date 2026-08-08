#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID              "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define AUDIO_CHAR_UUID        "66666666-6666-6666-6666-666666666666"

#define MIC_PIN                4
#define SAMPLE_RATE            16000
#define SAMPLES_PER_PACKET     64
#define ADC_RESOLUTION_BITS    12

extern HardwareSerial Serial;

BLEServer *pServer = NULL;
BLECharacteristic *pChar = NULL;
BLECharacteristic *pAudioChar = NULL;
bool deviceConnected = false;
uint32_t counter = 0;
uint32_t audioCounter = 0;

int16_t audioBuffer[SAMPLES_PER_PACKET];

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
        Serial.println("[提示] 手机已连接！");
    }

    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
        Serial.println("[提示] 手机已断开");
        BLEDevice::startAdvertising();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            Serial.print("收到数据: ");
            Serial.println(value.c_str());
        }
    }
};

void initMicrophone() {
    analogReadResolution(12);
    // set attenuation on the microphone pin (preferred API)
    #ifdef analogSetPinAttenuation
    analogSetPinAttenuation(MIC_PIN, ADC_11db);
    #else
    analogSetAttenuation(ADC_11db);
    #endif
    pinMode(MIC_PIN, INPUT);
    Serial.printf("麦克风初始化完成，采样率: %d Hz\n", SAMPLE_RATE);
    
    // 诊断：测试 ADC 读数
    Serial.println("ADC 诊断测试（请对着麦克风说话或触摸 GPIO4）：");
    for (int i = 0; i < 20; i++) {
        int raw = analogRead(MIC_PIN);
        Serial.printf("ADC[%d]: %d\n", i, raw);
        delay(100);
    }
}

int readAudio(int16_t *samples, int count) {
    const uint32_t periodUs = 1000000UL / SAMPLE_RATE; // truncated microseconds
    const int adcMax = (1 << ADC_RESOLUTION_BITS); // e.g. 4096 for 12-bit
    const int adcMid = adcMax / 2; // center value

    uint32_t nextMicros = micros();
    for (int i = 0; i < count; i++) {
        // wait until next sample time
        nextMicros += periodUs;
        while ((int32_t)(micros() - nextMicros) < 0) {
            // busy wait for precise timing
        }

        int raw = analogRead(MIC_PIN);
        // convert unsigned ADC reading to signed 16-bit centered value
        int32_t centered = ((int32_t)raw - adcMid) << (16 - ADC_RESOLUTION_BITS);
        samples[i] = (int16_t)centered;
    }
    return count;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP32-C3 BLE ADC 麦克风 ---");

    initMicrophone();

    BLEDevice::init("esp32");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pChar = pService->createCharacteristic(
        CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pChar->setCallbacks(new MyCallbacks());
    pChar->setValue("esp32 mic ready");
    pChar->addDescriptor(new BLE2902());

    pAudioChar = pService->createCharacteristic(
        AUDIO_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pAudioChar->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();

    Serial.println("蓝牙启动成功！设备名：esp32");
    Serial.println("开始主循环...");
}

void loop() {
    if (deviceConnected) {
        counter++;

        if (counter % 10 == 0) {
            char buf[32];
            snprintf(buf, sizeof(buf), "count:%lu", counter);
            pChar->setValue(buf);
            pChar->notify();
        }

        int16_t samples[SAMPLES_PER_PACKET];
        int samplesRead = readAudio(samples, SAMPLES_PER_PACKET);

        if (samplesRead > 0) {
            pAudioChar->setValue((uint8_t *)samples, samplesRead * sizeof(int16_t));
            pAudioChar->notify();
            audioCounter++;

            if (audioCounter % 100 == 0) {
                Serial.printf("已发送 %lu 包音频数据\n", audioCounter);
            }
        }
    } else {
        delay(100);
    }
}
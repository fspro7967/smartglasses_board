#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// ===== BLE UUID 定义 =====
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID              "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define AUDIO_CHAR_UUID        "66666666-6666-6666-6666-666666666666"
#define PLAYBACK_CHAR_UUID     "88888888-8888-8888-8888-888888888888"  // 新增：接收音频特征值

// ===== 麦克风配置 =====
#define MIC_PIN                4
#define SAMPLE_RATE            16000
#define SAMPLES_PER_PACKET     64
#define ADC_RESOLUTION_BITS    12

// ===== 扬声器播放配置 =====
#define SPEAKER_PIN            0          // 扬声器PWM引脚
#define PWM_FREQUENCY          20000      // PWM载波频率（人耳听不见）
#define PWM_RESOLUTION         8          // 8位分辨率（0-255）
#define PLAYBACK_BUFFER_SIZE   512        // 播放缓冲区大小（样本数）

// ===== 播放状态 =====
#define VOLUME_MIN             0
#define VOLUME_MAX             255
#define VOLUME_DEFAULT         180

extern HardwareSerial Serial;

BLEServer *pServer = NULL;
BLECharacteristic *pChar = NULL;
BLECharacteristic *pAudioChar = NULL;
BLECharacteristic *pPlaybackChar = NULL;   // 新增：播放特征值

bool deviceConnected = false;
uint32_t counter = 0;
uint32_t audioCounter = 0;

int16_t audioBuffer[SAMPLES_PER_PACKET];

// ===== 播放相关变量 =====
volatile bool isPlaying = false;
volatile uint8_t volume = VOLUME_DEFAULT;
int16_t playbackBuffer[PLAYBACK_BUFFER_SIZE];
volatile int playbackWriteIndex = 0;
volatile int playbackReadIndex = 0;
volatile int playbackAvailable = 0;
uint32_t lastPlayTime = 0;
const uint32_t playIntervalUs = 1000000UL / SAMPLE_RATE;

// ===== BLE 服务器回调 =====
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

// ===== 控制特征值回调（接收指令） =====
class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            String cmd = String(value.c_str());
            Serial.print("收到指令: ");
            Serial.println(cmd);
            
            // ---- 解析控制指令 ----
            if (cmd == "play") {
                isPlaying = true;
                playbackReadIndex = 0;
                playbackAvailable = 0;
                Serial.println("▶ 开始播放");
                pCharacteristic->setValue("OK:playing");
                pCharacteristic->notify();
            }
            else if (cmd == "stop") {
                isPlaying = false;
                playbackWriteIndex = 0;
                playbackReadIndex = 0;
                playbackAvailable = 0;
                ledcWrite(0, 0);  // 静音
                Serial.println("⏹ 停止播放");
                pCharacteristic->setValue("OK:stopped");
                pCharacteristic->notify();
            }
            else if (cmd.startsWith("volume:")) {
                int vol = cmd.substring(7).toInt();
                if (vol >= 0 && vol <= 100) {
                    volume = map(vol, 0, 100, 0, 255);
                    Serial.printf("🔊 音量: %d%%\n", vol);
                    pCharacteristic->setValue(("OK:volume:" + String(vol)).c_str());
                    pCharacteristic->notify();
                } else {
                    pCharacteristic->setValue("ERROR:volume 0-100");
                    pCharacteristic->notify();
                }
            }
            else if (cmd == "status") {
                String status = isPlaying ? "playing" : "stopped";
                int volPercent = map(volume, 0, 255, 0, 100);
                String response = "status:" + status + ",volume:" + String(volPercent) + 
                                  ",buffer:" + String(playbackAvailable);
                pCharacteristic->setValue(response.c_str());
                pCharacteristic->notify();
                Serial.println("📊 状态已发送");
            }
            else if (cmd == "echo") {
                pCharacteristic->setValue("echo:hello from ESP32");
                pCharacteristic->notify();
                Serial.println("🔄 回声响应");
            }
            else {
                Serial.println("⚠ 未知指令");
                pCharacteristic->setValue("ERROR:unknown command");
                pCharacteristic->notify();
            }
        }
    }
};

// ===== 播放控制特征值回调（接收音频数据） =====
class PlaybackCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        // 如果不处于播放模式，忽略数据
        if (!isPlaying) {
            return;
        }
        
        // 获取音频数据
        std::string data = pCharacteristic->getValue();
        int dataLen = data.length();
        
        if (dataLen == 0) return;
        
        // 计算可复制的样本数
        int samplesCount = dataLen / sizeof(int16_t);
        int16_t* samples = (int16_t*)data.data();
        
        // 写入环形缓冲区
        for (int i = 0; i < samplesCount; i++) {
            int nextIndex = (playbackWriteIndex + 1) % PLAYBACK_BUFFER_SIZE;
            
            // 检查缓冲区是否已满
            if (nextIndex == playbackReadIndex) {
                // 缓冲区满，丢弃新数据
                break;
            }
            
            playbackBuffer[playbackWriteIndex] = samples[i];
            playbackWriteIndex = nextIndex;
            playbackAvailable++;
        }
    }
};

// ===== 初始化麦克风 =====
void initMicrophone() {
    analogReadResolution(ADC_RESOLUTION_BITS);
    #ifdef analogSetPinAttenuation
    analogSetPinAttenuation(MIC_PIN, ADC_11db);
    #else
    analogSetAttenuation(ADC_11db);
    #endif
    pinMode(MIC_PIN, INPUT);
    Serial.printf("麦克风初始化完成，采样率: %d Hz\n", SAMPLE_RATE);
    
    Serial.println("ADC 诊断测试（请对着麦克风说话或触摸 GPIO4）：");
    for (int i = 0; i < 20; i++) {
        int raw = analogRead(MIC_PIN);
        Serial.printf("ADC[%d]: %d\n", i, raw);
        delay(100);
    }
}

// ===== 初始化扬声器（PWM） =====
void initSpeaker() {
    ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(SPEAKER_PIN, 0);
    ledcWrite(0, 0);  // 初始静音
    Serial.println("扬声器初始化完成 (PWM模式)");
}

// ===== 播放单个PCM样本 =====
void playSample(int16_t sample) {
    // 将16位有符号样本映射到8位PWM值 (0-255)
    // sample范围: -32768 ~ 32767
    uint32_t pwmValue = (uint32_t)(sample + 32768) * 255 / 65535;
    
    // 应用音量
    pwmValue = (pwmValue * volume) / 255;
    
    // 输出PWM
    ledcWrite(0, pwmValue);
}

// ===== 处理播放队列 =====
void processPlayback() {
    if (!isPlaying) {
        // 非播放状态，确保静音
        ledcWrite(0, 0);
        return;
    }
    
    // 检查缓冲区是否有数据
    if (playbackAvailable == 0 || playbackReadIndex == playbackWriteIndex) {
        // 缓冲区空，输出静音（避免噪声）
        ledcWrite(0, 0);
        return;
    }
    
    // 读取一个样本
    int16_t sample = playbackBuffer[playbackReadIndex];
    playbackReadIndex = (playbackReadIndex + 1) % PLAYBACK_BUFFER_SIZE;
    playbackAvailable--;
    
    // 播放样本
    playSample(sample);
}

// ===== 读取麦克风音频 =====
int readAudio(int16_t *samples, int count) {
    const uint32_t periodUs = 1000000UL / SAMPLE_RATE;
    const int adcMax = (1 << ADC_RESOLUTION_BITS);
    const int adcMid = adcMax / 2;

    uint32_t nextMicros = micros();
    for (int i = 0; i < count; i++) {
        nextMicros += periodUs;
        while ((int32_t)(micros() - nextMicros) < 0) {
            // busy wait
        }

        int raw = analogRead(MIC_PIN);
        int32_t centered = ((int32_t)raw - adcMid) << (16 - ADC_RESOLUTION_BITS);
        samples[i] = (int16_t)centered;
    }
    return count;
}

// ===== 主初始化 =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP32-C3 BLE 音频收发器 ---");

    // 初始化麦克风
    initMicrophone();
    
    // 初始化扬声器
    initSpeaker();

    // 初始化 BLE
    BLEDevice::init("ESP32_Audio");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // ----- 控制特征值（收发指令） -----
    pChar = pService->createCharacteristic(
        CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pChar->setCallbacks(new MyCallbacks());
    pChar->setValue("ESP32 Audio Ready");
    pChar->addDescriptor(new BLE2902());

    // ----- 音频发送特征值（麦克风→手机） -----
    pAudioChar = pService->createCharacteristic(
        AUDIO_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pAudioChar->addDescriptor(new BLE2902());

    // ----- 音频接收特征值（手机→扬声器） 新增 -----
    pPlaybackChar = pService->createCharacteristic(
        PLAYBACK_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pPlaybackChar->setCallbacks(new PlaybackCallbacks());
    pPlaybackChar->addDescriptor(new BLE2902());

    pService->start();

    // 开始广播
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();

    Serial.println("========================================");
    Serial.println("蓝牙启动成功！设备名: ESP32_Audio");
    Serial.println("可用指令:");
    Serial.println("  play          - 开始播放");
    Serial.println("  stop          - 停止播放");
    Serial.println("  volume:50     - 设置音量 (0-100)");
    Serial.println("  status        - 查询状态");
    Serial.println("  echo          - 回声测试");
    Serial.println("========================================");
    Serial.println("开始主循环...");
}

// ===== 主循环 =====
void loop() {
    // ---- 发送音频（麦克风→手机） ----
    if (deviceConnected) {
        counter++;

        // 每10次发送心跳
        if (counter % 10 == 0) {
            char buf[32];
            snprintf(buf, sizeof(buf), "count:%lu", counter);
            pChar->setValue(buf);
            pChar->notify();
        }

        // 采集并发送音频数据
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
    
    // ---- 播放音频（手机→扬声器） ----
    // 每次循环处理一个样本，保持16kHz播放速率
    static uint32_t lastPlayMicros = 0;
    uint32_t now = micros();
    
    if (now - lastPlayMicros >= playIntervalUs) {
        lastPlayMicros = now;
        processPlayback();
    }
}
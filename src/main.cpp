#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <driver/i2s.h>
#include <math.h>

// ===== BLE UUID 定义 =====
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID              "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define AUDIO_CHAR_UUID        "66666666-6666-6666-6666-666666666666"
#define PLAYBACK_CHAR_UUID     "88888888-8888-8888-8888-888888888888"  // 新增：接收音频特征值

// ===== 音频采样配置 =====
#define SAMPLE_RATE            16000
#define SAMPLES_PER_PACKET     64

// ===== I2S 引脚定义 (ESP32-C3 只有一个 I2S 外设，麦克风与功放全双工共用) =====
// INMP441 (数字麦克风) 引脚  →  ESP32-C3 GPIO
//   SCK (BCLK)  →  GPIO0   （与 MAX98357 BCLK 共用）
//   WS  (LRCLK) →  GPIO1   （与 MAX98357 LRC  共用）
//   SD  (DOUT)  →  GPIO2
//   L/R         →  GND（选择左声道）
//   VDD         →  3.3V
//   GND         →  GND
//
// MAX98357 (I2S 数字功放) 引脚 →  ESP32-C3 GPIO
//   BCLK        →  GPIO0   （与 INMP441 SCK 共用）
//   LRC         →  GPIO1   （与 INMP441 WS  共用）
//   DIN         →  GPIO3
//   VIN         →  5V / 3.3V
//   GND         →  GND
//   SD          →  VIN（选择左声道并保持使能）
//   GAIN        →  悬空（9dB）
//   扬声器      →  接 + 和 -（BTL 输出，任一端不可接地）
#define I2S_PORT               I2S_NUM_0      // ESP32-C3 只有 1 个 I2S 外设
#define I2S_BCK_PIN            0              // INMP441 SCK / MAX98357 BCLK
#define I2S_WS_PIN             2              // INMP441 WS  / MAX98357 LRC
#define I2S_SD_PIN             1              // INMP441 SD  (数据输出)
#define I2S_DOUT_PIN           3              // MAX98357 DIN (数据输入)

// ===== 麦克风存在性检测 =====
// 思路：INMP441 即使在没有声音时也会输出非零的本底噪声（约 1~2 LSB），
// 交流 RMS 明显大于 0；而数据线悬空（未接麦克风）时被内部上拉/下拉钳死，
// 16 位样本几乎完全不变。因此用「一个短窗口内的交流 RMS + 是否恒定」作为判据。
// 注意：这只识别“数据线没有被麦克风驱动”。悬空且随机翻转的噪声线仍可能被
// 误判为存在，属于该方法的固有局限。
#define MIC_DETECT_WINDOW_SAMPLES  1600    // 判定窗口：100ms @16kHz
#define MIC_WARMUP_SAMPLES         800     // 启动后先丢弃的样本数（50ms），等 I2S 稳定
#define MIC_AC_RMS_THRESHOLD       0.5f    // 交流 RMS 阈值（LSB），实测环境过吵/过静时可调

// ===== 扬声器播放配置（MAX98357 I2S 数字功放，与麦克风共用 I2S0 全双工） =====
#define PLAYBACK_BUFFER_SIZE   512        // 接收环形缓冲区大小（样本数）
#define PLAYBACK_CHUNK         128        // 每次写入 I2S 的最大样本数

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

// ===== 麦克风存在性状态 =====
enum MicState { MIC_UNKNOWN, MIC_PRESENT, MIC_ABSENT };
volatile MicState micState = MIC_UNKNOWN;
int16_t micDetectBuffer[MIC_DETECT_WINDOW_SAMPLES];  // 判定窗口累积缓冲
int micDetectCount = 0;                              // 当前窗口已累积的样本数
int micWarmupRemaining = MIC_WARMUP_SAMPLES;         // 启动后剩余待丢弃的样本数

const char* micStateStr() {
    switch (micState) {
        case MIC_PRESENT: return "present";
        case MIC_ABSENT:  return "absent";
        default:          return "unknown";
    }
}

// ===== 播放相关变量 =====
volatile bool isPlaying = false;
volatile uint8_t volume = VOLUME_DEFAULT;
int16_t playbackBuffer[PLAYBACK_BUFFER_SIZE];
volatile int playbackWriteIndex = 0;
volatile int playbackReadIndex = 0;
volatile int playbackAvailable = 0;
int16_t txBuffer[PLAYBACK_CHUNK];

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
                                  ",buffer:" + String(playbackAvailable) +
                                  ",mic:" + micStateStr();
                pCharacteristic->setValue(response.c_str());
                pCharacteristic->notify();
                Serial.println("状态已发送");
            }
            else if (cmd == "echo") {
                pCharacteristic->setValue("echo:hello from ESP32");
                pCharacteristic->notify();
                Serial.println("回声响应");
            }
            else {
                Serial.println("未知指令");
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

// ===== 初始化 I2S（INMP441 采集 + MAX98357 播放，I2S0 全双工） =====
void initI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),  // 全双工
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,   // L/R 接 GND = 左声道
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = SAMPLES_PER_PACKET,
        .use_apll = false,
        .tx_desc_auto_clear = true,   // 播放欠载时自动输出静音，避免重复旧数据
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCK_PIN,      // INMP441 SCK / MAX98357 BCLK
        .ws_io_num = I2S_WS_PIN,        // INMP441 WS  / MAX98357 LRC
        .data_out_num = I2S_DOUT_PIN,   // MAX98357 DIN
        .data_in_num = I2S_SD_PIN       // INMP441 SD (DOUT)
    };

    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[错误] I2S 驱动安装失败: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[错误] I2S 引脚设置失败: %d\n", err);
        return;
    }

    i2s_start(I2S_PORT);

    Serial.printf("I2S 全双工初始化完成，采样率: %d Hz\n", SAMPLE_RATE);
    Serial.printf("  麦克风 INMP441: BCK=GPIO%d WS=GPIO%d SD=GPIO%d\n",
                  I2S_BCK_PIN, I2S_WS_PIN, I2S_SD_PIN);
    Serial.printf("  功放 MAX98357: BCLK=GPIO%d LRC=GPIO%d DIN=GPIO%d\n",
                  I2S_BCK_PIN, I2S_WS_PIN, I2S_DOUT_PIN);
}

// ===== 处理播放队列：从环形缓冲区取出样本，应用音量后写入 I2S =====
void processPlayback() {
    if (!isPlaying) {
        return;   // TX 欠载时由 tx_desc_auto_clear 自动输出静音
    }

    int count = 0;
    while (count < PLAYBACK_CHUNK && playbackAvailable > 0 &&
           playbackReadIndex != playbackWriteIndex) {
        int16_t sample = playbackBuffer[playbackReadIndex];
        playbackReadIndex = (playbackReadIndex + 1) % PLAYBACK_BUFFER_SIZE;
        playbackAvailable--;

        int32_t scaled = (int32_t)sample * volume / VOLUME_MAX;
        txBuffer[count++] = (int16_t)scaled;
    }

    if (count == 0) {
        return;
    }

    size_t bytesWritten = 0;
    i2s_write(I2S_PORT, txBuffer, count * sizeof(int16_t), &bytesWritten, portMAX_DELAY);
}

// ===== 读取 INMP441 原始音频（I2S，不做滤波） =====
int readRawAudio(int16_t *samples, int count) {
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT, samples, count * sizeof(int16_t),
                             &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) {
        return 0;
    }

    return bytesRead / sizeof(int16_t);
}

// ===== 直流偏置消除（INMP441 常见直流偏移，一阶 DC Blocker）=====
void applyDcBlocker(int16_t *samples, int count) {
    static int16_t prevSample = 0;
    static int32_t filteredState = 0;
    const int32_t R = 32580;   // ≈ 0.994 × 32768，高通截止约 15Hz @ 16kHz
    for (int i = 0; i < count; i++) {
        int32_t in = samples[i];
        int32_t out = in - prevSample + ((filteredState * R) >> 15);
        samples[i] = (int16_t)out;
        filteredState = out;
        prevSample = in;
    }
}

// ===== 判断一个窗口内是否真的接入了麦克风 =====
bool analyzeMicPresence(const int16_t *samples, int count) {
    if (count <= 0) {
        return false;
    }

    int16_t vmin = samples[0];
    int16_t vmax = samples[0];
    int64_t sum = 0;
    int64_t sumSq = 0;
    for (int i = 0; i < count; i++) {
        int32_t v = samples[i];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
        sum += v;
        sumSq += (int64_t)v * (int64_t)v;
    }

    // 数据线完全不动：没有器件在驱动它（上拉/下拉钳死或恒定输出）
    if (vmin == vmax) {
        return false;
    }

    // 去掉直流分量后的交流 RMS；真实麦克风的本底噪声会让它大于阈值
    double mean = (double)sum / (double)count;
    double acPower = (double)sumSq / (double)count - mean * mean;
    if (acPower < 0.0) {
        acPower = 0.0;
    }
    double acRms = sqrt(acPower);
    return acRms >= MIC_AC_RMS_THRESHOLD;
}

void updateMicState(bool present) {
    MicState newState = present ? MIC_PRESENT : MIC_ABSENT;
    if (newState == micState) {
        return;
    }
    micState = newState;
    if (micState == MIC_PRESENT) {
        Serial.println("[提示] 已检测到麦克风，恢复音频发送");
    } else {
        Serial.println("[警告] 未检测到麦克风，已暂停音频发送");
    }
}

// 累积原始样本，凑满一个判定窗口后更新麦克风状态。
// 用原始（未去直流）样本判定，避免 DC Blocker 把恒定输入抹成 0 后掩盖真实电平。
void updateMicPresence(const int16_t *samples, int count) {
    if (micWarmupRemaining > 0) {
        int skip = (count < micWarmupRemaining) ? count : micWarmupRemaining;
        samples += skip;
        count -= skip;
        micWarmupRemaining -= skip;
        if (count <= 0) {
            return;
        }
    }

    while (count > 0) {
        int space = MIC_DETECT_WINDOW_SAMPLES - micDetectCount;
        int take = (count < space) ? count : space;
        memcpy(&micDetectBuffer[micDetectCount], samples, take * sizeof(int16_t));
        micDetectCount += take;
        samples += take;
        count -= take;

        if (micDetectCount >= MIC_DETECT_WINDOW_SAMPLES) {
            micDetectCount = 0;
            updateMicState(analyzeMicPresence(micDetectBuffer, MIC_DETECT_WINDOW_SAMPLES));
        }
    }
}


// ===== 主初始化 =====
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP32-C3 BLE 音频收发器 ---");

    // 初始化 I2S（INMP441 采集 + MAX98357 播放，I2S0 全双工）
    initI2S();

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
    Serial.printf("麦克风检测: 窗口 %d 样本, 交流 RMS 阈值 %.2f\n",
                  MIC_DETECT_WINDOW_SAMPLES, MIC_AC_RMS_THRESHOLD);
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

        // 采集音频数据
        int16_t rawSamples[SAMPLES_PER_PACKET];
        int samplesRead = readRawAudio(rawSamples, SAMPLES_PER_PACKET);

        if (samplesRead > 0) {
            // 先用原始样本判断麦克风是否接入
            updateMicPresence(rawSamples, samplesRead);

            // 再做直流偏置消除准备发送
            applyDcBlocker(rawSamples, samplesRead);

            // 只有确认麦克风已接入才发送，避免把静音/悬空数据喂给手机端 Whisper
            if (micState == MIC_PRESENT) {
                pAudioChar->setValue((uint8_t *)rawSamples, samplesRead * sizeof(int16_t));
                pAudioChar->notify();
                audioCounter++;

                if (audioCounter % 100 == 0) {
                    Serial.printf("已发送 %lu 包音频数据\n", audioCounter);
                }
            }
        }
    } else {
        delay(100);
    }
    
    // ---- 播放音频（手机→MAX98357） ----
    // i2s_write 按 DMA 速率阻塞，自动保持 16kHz 播放
    processPlayback();
}
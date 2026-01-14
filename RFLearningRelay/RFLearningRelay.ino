#include <EEPROM.h>
#include "RFDecoder.h" //https://github.com/nestik83/RFDecoder

#if defined(ESP8266) || defined(ESP32)
#define USE_WIFI 1  // 1 на esp можно включить wi-fi, 0 - для ардуино без WI-FI
#endif

//================= WIFI SETTINGS =============================
#if USE_WIFI
//#include <WiFi.h>
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoMqttClient.h>
#include <WiFiManager.h>

const char broker[] = "m2.wqtt.ru";
int port = 16017;
String topic = "stat/ESP32-test";
const char *mqttUser = "user"; 
const char *mqttPass = "pass";
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastWifiReconnectAttempt = 0;

WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient mqttNetClient;
MqttClient mqttClient(mqttNetClient);
WiFiManager wm;
#endif
//===========================================================

//================= PINS SETTINGS ===========================
#define USE_RGB_LED 0       // 1 - использовать RGB, 0 - обычный LED
#define COMMON_ANODE_LED 1  // 1 - общий анод, 0 - общий катод

#if defined(ESP32)
#define MAX_EEPROM_SIZE 2048
#define CHANNEL_COUNT 4  //количество каналов
#if USE_RGB_LED
#define LED_R 18
#define LED_G 19
#define LED_B 16
#else
#define LED_PIN 16  //13
#endif
#define RF_PIN 27      //2
#define BUTTON_PIN 17  //4
const uint8_t channelPins[CHANNEL_COUNT] = { 21, 22, 23, 25 };
const uint8_t buttonPins[CHANNEL_COUNT] = {26, 27, 32, 33};

#elif defined(ESP8266)
#define MAX_EEPROM_SIZE 2048
#define CHANNEL_COUNT 2  //количество каналов
#if USE_RGB_LED
#define LED_R 2
#define LED_G 2
#define LED_B 2
#else
#define LED_PIN 2
#endif
#define RF_PIN 4    
#define BUTTON_PIN 5
const uint8_t channelPins[CHANNEL_COUNT] = {14, 16};
const uint8_t buttonPins[CHANNEL_COUNT] = {12, 13};

#else
#define MAX_EEPROM_SIZE 1024
#define CHANNEL_COUNT 4  //количество каналов
#if USE_RGB_LED
#define LED_R 13
#define LED_G 13
#define LED_B 13
#else
#define LED_PIN 13  //13
#endif
#define RF_PIN 2      //2
#define BUTTON_PIN 3  //4
const uint8_t channelPins[CHANNEL_COUNT] = { 4, 5, 6, 7 };
#endif
//===========================================================

#define MAX_RECIEVE_BUFFER 10  //Максимальный буфер приема байтов по RF каналу
#define PRESS_TIMEOUT 1000
#define LEARN_TIMEOUT 15000
#define HOLD_TIMEOUT 5000
#define LED_BLINK_DELAY 300
#define SERIAL_BUFFER_SIZE 32
#define MODE_MASK 0xE0
#define STATE_MASK 0x01
#define MODE_SHIFT 5
#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 5000
#define R 1
#define G 2
#define B 3
#define MODE_BOTH_EDGES   1   // реагируем на все фронты
#define MODE_FALLING_ONLY 2   // реагируем только на спад (нажатие)
#define MODE_LEVEL_TOGGLE 3   // LOW = ВКЛ, HIGH = ВЫКЛ
// Настройка: реагировать только на спад или на оба фронта
uint8_t triggerMode = MODE_BOTH_EDGES;
String serialCommand = "";
uint8_t channelModes[CHANNEL_COUNT];
unsigned long lastTrigger[CHANNEL_COUNT] = { 0 };
bool channelState[CHANNEL_COUNT] = { false };
bool prevChannelState[CHANNEL_COUNT] = { false };
bool ignoreInput[CHANNEL_COUNT] = { false };
unsigned long ignoreUntil[CHANNEL_COUNT] = { 0 };
uint8_t channelMode[CHANNEL_COUNT] = { 0 };  // активный режим

bool btnLastState[CHANNEL_COUNT];
bool btnCurrentState[CHANNEL_COUNT];
unsigned long btnLastChangeTime[CHANNEL_COUNT];


unsigned long lastPressTime = 0;
uint8_t pressCount = 0;
bool learningChannel = false;
bool learningMode = false;
unsigned long learnStartTime = 0;
uint8_t learnChannel = 0;
uint8_t learnMode = 0;
unsigned long previousMillis = 0;  // время последнего переключения
bool ledState = false;

RFDecoder rf(RF_PIN, MAX_RECIEVE_BUFFER);


//===================== WIFI Telnet MQTT =======================

#if USE_WIFI
void remotePrint(const String &msg) {
  //Serial.print(msg);
  if (telnetClient && telnetClient.connected()) telnetClient.print(msg);
}

void remotePrintln(const String &msg) {
  //Serial.println(msg);
  if (telnetClient && telnetClient.connected()) telnetClient.println(msg);
}

void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();
  String payload;

  while (mqttClient.available()) {
    payload += (char)mqttClient.read();
  }

  payload.trim();
  payload.toUpperCase();

  Serial.print("📩 MQTT message:\n  Topic: ");
  Serial.println(topic);
  Serial.print("  Payload: ");
  Serial.println(payload);

  // Проверяем, что топик соответствует шаблону cmnd/ESP-32test/CHANNELX
  if (topic.startsWith("cmnd/ESP-32test/CHANNEL")) {
    int channel = topic.substring(strlen("cmnd/ESP-32test/CHANNEL")).toInt();

    uint8_t mode;
    if (payload == "ON") mode = 3;
    else if (payload == "OFF") mode = 4;
    else if (payload == "KEEP") mode = 2;
    else if (payload == "TOGGLE") mode = 1;
    else {
      Serial.println("Unknown command");
      return;
    }

    // вызываем твою функцию
    processChannel(channel, mode);
  }
}
#endif
// ===================== ОСНОВНЫЕ ФУНКЦИИ =====================

// mode:
// 1 – переключение состояния с антидребезгом (игнор 500 мс)
// 2 – поддержание включения при частых вызовах (таймер 1000 мс)
// 3 – включить
// 4 – выключить

// ============================= LED =============================


String getChipID() {
#if defined(ESP8266)
  // ESP8266: возвращает 32-битный ID (обычно последние 3 байта MAC)
  uint32_t id = ESP.getChipId();
  char buf[9];
  sprintf(buf, "%06lX", id & 0xFFFFFF); // 6 hex digits, upper-case, zero-padded
  return String(buf);
#elif defined(ESP32)
  // ESP32: берём MAC (64-bit в efuse), но хотим последние 3 байта (как ESP8266)
  uint64_t mac = ESP.getEfuseMac();      // MAC в правильном порядке
  uint32_t id = (uint32_t)(mac & 0xFFFFFFUL); // последние 3 байта
  char buf[9];
  sprintf(buf, "%06lX", id); // 6 hex digits, upper-case, zero-padded
  return String(buf);
#else
  return String("UNKNOWN");
#endif
}

void led(bool state, uint8_t color = B) {
  if (COMMON_ANODE_LED) {
#if USE_RGB_LED
    if (color == R) {
      digitalWrite(LED_R, !state);
    } else if (color == G) {
      digitalWrite(LED_G, !state);
    } else if (color == B) {
      digitalWrite(LED_B, !state);
    }
#else
    digitalWrite(LED_PIN, !state);
#endif
  } else {
#if USE_RGB_LED
    if (color == R) {
      digitalWrite(LED_R, state);
    } else if (color == G) {
      digitalWrite(LED_G, state);
    } else if (color == B) {
      digitalWrite(LED_B, state);
    }
#else
    digitalWrite(LED_PIN, state);
#endif
  }
}

void blinkNonBlocking(unsigned long interval) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    led(ledState);
  }
}


void blinkLed(uint8_t times, unsigned int delayMs = LED_BLINK_DELAY) {
  for (uint8_t i = 0; i < times; i++) {
    led(HIGH);
    delay(delayMs);
    led(LOW);
    delay(delayMs);
  }
}

// --- Вспомогательные функции ---
bool waitButtonPress() {
  static uint32_t lastPress = 0;
  if (digitalRead(BUTTON_PIN) == LOW) {  // кнопка нажата
    if (millis() - lastPress > DEBOUNCE_MS) {
      lastPress = millis();
      return true;  // фронт нажатия с защитой от дребезга
    }
  }
  return false;
}

bool detectLongPress(uint32_t &startTime) {
  if (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - startTime >= LONG_PRESS_MS) {
      return true;  // длинное удержание
    }
  } else {
    startTime = millis();  // сброс таймера при отпускании
  }
  return false;
}

uint8_t readChannelMode(uint8_t ch) {
  uint8_t val = EEPROM.read(MAX_EEPROM_SIZE - CHANNEL_COUNT + ch);
  return (val & MODE_MASK) >> MODE_SHIFT;
}

bool readChannelState(uint8_t ch) {
  uint8_t val = EEPROM.read(MAX_EEPROM_SIZE - CHANNEL_COUNT + ch);
  return val & STATE_MASK;
}

void writeChannelEEPROM(uint8_t ch, uint8_t mode, bool state) {
  uint8_t val = ((mode & 0x07) << MODE_SHIFT) | (state ? 1 : 0);
  EEPROM.write(MAX_EEPROM_SIZE - CHANNEL_COUNT + ch, val);
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
}

void ledOn(uint16_t durationMs) {
  led(HIGH);
  delay(durationMs);
  led(LOW);
}

bool buttonPressed() {
  static uint8_t lastState = HIGH;
  static uint32_t lastDebounceTime = 0;

  uint8_t reading = digitalRead(BUTTON_PIN);
  if (reading != lastState) {
    lastDebounceTime = millis();
    lastState = reading;
  }
  if (reading == LOW && (millis() - lastDebounceTime) > DEBOUNCE_MS) {
    return true;
  }
  return false;
}

bool waitLongPress() {
  uint32_t pressTime = millis();
  while (digitalRead(BUTTON_PIN) == LOW) {
    if (millis() - pressTime >= LONG_PRESS_MS) return true;
  }
  return false;
}

// --- Режим обучения каналов ---
void trainingMode() {
  Serial.println(F("=== TRAINING MODE ==="));
  ledOn(2000);  // 2 секунды горит при входе в обучение

  // --- Этап 1: выбор канала ---
  uint8_t currentChannel = 0;
  while (true) {
    if (buttonPressed()) {
      if (waitLongPress()) {
        Serial.print(F("Channel confirmed: "));
        Serial.println(currentChannel);
        ledOn(2000);  // подтверждение
        break;
      } else {
        currentChannel = (currentChannel + 1) % CHANNEL_COUNT;
        Serial.print(F("Select channel: "));
        Serial.println(currentChannel);
        blinkLed(currentChannel + 1, 300);
      }
    }
  }

  // --- Этап 2: выбор режима ---
  uint8_t mode = readChannelMode(currentChannel);
  if (mode < 1 || mode > 3) mode = 1;

  while (true) {
    if (buttonPressed()) {
      if (waitLongPress()) {
        Serial.print(F("Mode confirmed: "));
        Serial.println(mode);
        writeChannelEEPROM(currentChannel, mode, 0);
        ledOn(2000);  // подтверждение
        Serial.println(F("Training complete."));
        break;
      } else {
        mode++;
        if (mode > 3) mode = 1;
        Serial.print(F("Select mode: "));
        Serial.println(mode);
        blinkLed(mode, 300);
      }
    }
  }
}

// --- Инициализация каналов при старте ---
void initChannels() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    uint8_t mode = readChannelMode(i);
    bool state = readChannelState(i);
    channelModes[i] = mode;
    pinMode(channelPins[i], OUTPUT);

    if (mode == 1) {
      digitalWrite(channelPins[i], state ? HIGH : LOW);
    } else if (mode == 2) {
      digitalWrite(channelPins[i], LOW);
    } else if (mode == 3) {
      digitalWrite(channelPins[i], HIGH);
    }
  }
}


void logState(uint8_t ch, const char *action) {
  // Проверяем, изменилось ли состояние
  if (channelState[ch] != prevChannelState[ch]) {
    prevChannelState[ch] = channelState[ch];  // обновляем запись

#if USE_WIFI

    if (mqttClient.connected()) {
      String stopic = "stat/ESP-32test/CHANNEL";
      stopic += ch;
      mqttClient.beginMessage(stopic);
      mqttClient.print(channelState[ch] ? "ON" : "OFF");
      mqttClient.endMessage();
    }
    
    remotePrint("CH");
    remotePrint(String(ch));
    remotePrint(" -> ");
    remotePrint(action);
    remotePrint(" | STATE: ");
    remotePrint(channelState[ch] ? "ON" : "OFF");
    remotePrint(" | MODE: ");
    remotePrintln(String(channelMode[ch]));
#endif

    Serial.print("CH");
    Serial.print(ch);
    Serial.print(" -> ");
    Serial.print(action);
    Serial.print(" | STATE: ");
    Serial.print(channelState[ch] ? "ON" : "OF");
    Serial.print(" | MODE: ");
    Serial.println(channelMode[ch]);
  }
}

void processChannel(uint8_t channel, uint8_t mode) {
  if (channel >= CHANNEL_COUNT) return;
  unsigned long now = millis();
  channelMode[channel] = mode;

  switch (mode) {
    case 2:  // поддержание включения
      channelState[channel] = true;
      lastTrigger[channel] = now;
      digitalWrite(channelPins[channel], HIGH);
      logState(channel, "KEEPON");
      break;

    case 1:  // переключение с защитой
      if (ignoreInput[channel] && now < ignoreUntil[channel]) {
        //Serial.print("CH"); Serial.print(channel);
        //Serial.println(" -> Ignored (debounce)");
        return;
      }
      ignoreInput[channel] = true;
      ignoreUntil[channel] = now + 500;
      channelState[channel] = !channelState[channel];
      digitalWrite(channelPins[channel], channelState[channel] ? HIGH : LOW);
      saveChannelStateIfNeeded(channel);
      logState(channel, "TOGGLE");
      break;

    case 3:  // включить
      channelState[channel] = true;
      digitalWrite(channelPins[channel], HIGH);
      saveChannelStateIfNeeded(channel);
      logState(channel, "STATON");
      break;

    case 4:  // выключить
      channelState[channel] = false;
      digitalWrite(channelPins[channel], LOW);
      saveChannelStateIfNeeded(channel);
      logState(channel, "STATOF");
      break;
  }
}

void updateOutputs() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    // автоотключение только для режима 2
    if (channelMode[i] == 2 && channelState[i] && (now - lastTrigger[i] > 400)) {
      channelState[i] = false;
      digitalWrite(channelPins[i], LOW);
      logState(i, "KEEPOF");
    }

    // сброс блокировки режима 2
    if (ignoreInput[i] && now > ignoreUntil[i]) {
      ignoreInput[i] = false;
      //Serial.print("CH"); Serial.print(i);
      //Serial.println(" -> Debounce released");
    }
  }
}

//====================== CHANNEL BUTTONS ============================

void handleChannelButtons() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    bool reading = digitalRead(buttonPins[i]);

    // обнаружено изменение уровня — возможный дребезг
    if (reading != btnLastState[i]) {
      btnLastChangeTime[i] = now;
      btnLastState[i] = reading;
    }

    // подтверждаем изменение после истечения времени дребезга
    if ((now - btnLastChangeTime[i]) > DEBOUNCE_MS) {
      if (reading != btnCurrentState[i]) {
        btnCurrentState[i] = reading;

        switch (triggerMode) {
          case MODE_BOTH_EDGES:
            // реагируем и на нажатие, и на отпускание
            processChannel(i, 1);
            break;

          case MODE_FALLING_ONLY:
            // реагируем только на спад (LOW)
            if (btnCurrentState[i] == LOW)
              processChannel(i, 1);
            break;

          case MODE_LEVEL_TOGGLE:
            // LOW = включить, HIGH = выключить
            if (btnCurrentState[i] == LOW)
              processChannel(i, 3); // включить
            else
              processChannel(i, 4); // выключить
            break;
        }
      }
    }
  }
}

//===================================================================



// ============================= EEPROM =============================
//
// [0] = количество записей
// [1..] = записи вида: [размер][канал][данные...]
//

void saveChannelStateIfNeeded(uint8_t channel) {
  uint16_t eepromAddr = (MAX_EEPROM_SIZE - CHANNEL_COUNT) + channel;
  uint8_t val = EEPROM.read(eepromAddr);
  Serial.print("startupeeprom-");
  Serial.println(val);
  uint8_t startupMode = (val >> 5) & 0x07;  // старшие биты — режим старта
  Serial.print("startupMode-");
  Serial.println(startupMode);
  if (startupMode != 1) return;  // 💾 сохраняем только если режим 1

  uint8_t newVal = (val & 0xFE) | (channelState[channel] & 0x01);
  if (newVal == val) return;

  EEPROM.write(eepromAddr, newVal);
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif

  Serial.print(F("EEPROM state saved for CH"));
  Serial.print(channel + 1);
  Serial.print(F(" (startup mode "));
  Serial.print(startupMode);
  Serial.print(F(") = "));
  Serial.println(channelState[channel] ? F("ON") : F("OFF"));
}


void dumpEEPROM() {
  Serial.println(F("EEPROM dump:"));
  for (uint32_t i = 0; i < MAX_EEPROM_SIZE; i++) {
    if (i % 16 == 0) {
      if (i != 0) Serial.println();
      if (i < 16) Serial.print("0");
      Serial.print(i, HEX);
      Serial.print(": ");
    }
    if (EEPROM.read(i) < 16) Serial.print("0");  // добавляем ведущий 0
    Serial.print(EEPROM.read(i), HEX);
    Serial.print(" ");
  }
  Serial.println();
}

uint8_t eepromCount() {
  return EEPROM.read(0);
}

void eepromSetCount(uint8_t count) {
  EEPROM.write(0, count);
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif
}

bool eepromSaveCode(uint8_t mode, uint8_t channel, const uint8_t *data, uint8_t size) {
  uint8_t count = eepromCount();
  int addr = 1;

  // найти конец списка
  for (uint8_t i = 0; i < count; i++) {
    if (addr >= MAX_EEPROM_SIZE - CHANNEL_COUNT) return false;
    uint8_t len = EEPROM.read(addr);
    if (len == 0xFF || len == 0) break;
    addr += len + 1;
  }

  // проверка на переполнение
  if (addr + size + 3 >= MAX_EEPROM_SIZE - CHANNEL_COUNT) {  // +3 канал и режим
    Serial.println(F("EEPROM full"));
    return false;
  }

  EEPROM.write(addr++, size + 2);  // +2 чтобы включить канал и режим в размер
  EEPROM.write(addr++, channel);
  EEPROM.write(addr++, mode);
  for (uint8_t i = 0; i < size; i++) EEPROM.write(addr++, data[i]);
  eepromSetCount(count + 1);

#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif

  Serial.print(F("Code saved for channel "));
  Serial.println(channel);
  return true;
}

bool eepromLoadCode(uint8_t index, uint8_t &mode, uint8_t &channel, uint8_t *out, uint8_t &size) {
  uint8_t count = eepromCount();
  if (index >= count) return false;

  int addr = 1;
  for (uint8_t i = 0; i < index; i++) {
    uint8_t len = EEPROM.read(addr);
    if (len == 0xFF || len == 0 || len > MAX_EEPROM_SIZE - CHANNEL_COUNT - addr) return false;
    addr += len + 1;
  }

  uint8_t len = EEPROM.read(addr++);
  channel = EEPROM.read(addr++);
  mode = EEPROM.read(addr++);
  size = len - 2;  // вычитаем байт канала и байт режима.
  if (size > MAX_EEPROM_SIZE - CHANNEL_COUNT - addr) return false;

  for (uint8_t i = 0; i < size; i++) out[i] = EEPROM.read(addr++);
  return true;
}

bool codesEqual(const uint8_t *a, const uint8_t *b, uint8_t len) {
  for (uint8_t i = 0; i < len; i++)
    if (a[i] != b[i]) return false;
  return true;
}

int eepromFindCode(const uint8_t *data, uint8_t size, uint8_t &channelOut, uint8_t &modeOut) {
  uint8_t buf[64];
  uint8_t chan, len, mode;
  uint8_t count = eepromCount();
  for (uint8_t i = 0; i < count; i++) {
    if (!eepromLoadCode(i, mode, chan, buf, len)) continue;
    if (len == size && codesEqual(buf, data, size)) {
      channelOut = chan;
      modeOut = mode;
      return i;
    }
  }
  return -1;
}

void eepromClear() {

  for (uint32_t i = 0; i < MAX_EEPROM_SIZE - CHANNEL_COUNT; i++) EEPROM.write(i, 0xFF);
  EEPROM.write(0, 0);
#if defined(ESP8266) || defined(ESP32)
  EEPROM.commit();
#endif

  Serial.println(F("EEPROM cleared"));
  blinkLed(10, 200);  // визуально показать стирание
}

// ============================= BUTTON =============================

void handleButton() {
  static bool prevStableState = HIGH;  // последнее стабильное состояние кнопки
  static bool lastRawState = HIGH;     // последнее считанное состояние
  static unsigned long lastDebounceTime = 0;
  static unsigned long pressStart = 0;

  bool rawState = digitalRead(BUTTON_PIN);
  unsigned long now = millis();

  // ===== Антидребезг =====
  if (rawState != lastRawState) {
    lastDebounceTime = now;  // обновляем таймер при изменении состояния
  }
  lastRawState = rawState;

  // если состояние стабильно более 50 мс — принимаем его за реальное
  if (now - lastDebounceTime > 50) {
    if (rawState != prevStableState) {
      prevStableState = rawState;

      // ===== Обработка нажатия =====
      if (rawState == LOW) {
        pressStart = now;  // начало удержания
      }

      // ===== Обработка отпускания =====
      else if (rawState == HIGH) {
        unsigned long pressTime = now - pressStart;
        learnStartTime = millis();

        if (pressTime >= 5000) {
          Serial.println(F("Long press - clearing EEPROM..."));
          eepromClear();
          learningChannel = false;
          pressCount = 0;
        } else {
          if (now - lastPressTime < PRESS_TIMEOUT) pressCount++;
          else pressCount = 1;
          lastPressTime = now;

          if (pressCount >= 5) {
            learningChannel = true;
            learnStartTime = now;
            pressCount = 0;
            learnChannel = 1;
            learnMode = 1;
            Serial.println(F("Enter learning mode..."));
            blinkLed(4, 300);
            delay(1000);
          } else if (learningChannel) {
            learnChannel++;
            if (learnChannel > CHANNEL_COUNT) learnChannel = 1;
            led(LOW);
            delay(1000);
            blinkLed(learnChannel, 300);
            delay(1000);
          } else if (learningMode) {
            learnMode++;
            if (learnMode > 4) learnMode = 1;
            led(LOW);
            delay(1000);
            blinkLed(learnMode, 300);
            delay(1000);
          }
        }
      }
    }
  }
}

void parseSerialCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // Команда формата CxMy
  if (cmd.startsWith("C")) {
    int cIndex = cmd.indexOf('C');
    int mIndex = cmd.indexOf('M');

    if (mIndex > cIndex && mIndex != -1) {
      uint8_t channel = cmd.substring(cIndex + 1, mIndex).toInt();
      uint8_t mode = cmd.substring(mIndex + 1).toInt();

      Serial.print(F(">> Command received: CH="));
      Serial.print(channel);
      Serial.print(F(" MODE="));
      Serial.println(mode);

      processChannel(channel, mode);
    } else {
      Serial.println(F("Invalid format. Use CxMy (example: C1M3)"));
    }
  }

  // Команда состояния
  else if (cmd == "STATUS" || cmd == "S") {
    printChannelStatus();
  }

  else if (cmd.startsWith("S")) {
    int sIndex = cmd.indexOf('S');
    uint8_t channel = cmd.substring(sIndex + 1).toInt();
    if (channel > CHANNEL_COUNT) {
      Serial.println("channel num > or < total channnel");
    } else {
      Serial.println(channelState[channel] ? "ON " : "OFF");
    }
  }

  else {
    Serial.println(F("Unknown command. Use format CxMy or STATUS"));
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    // Конец команды
    if (c == '\n' || c == '\r') {
      if (serialCommand.length() > 0) {
        parseSerialCommand(serialCommand);
        serialCommand = "";
      }
    }
    // Накопление символов
    else {
      if (serialCommand.length() < SERIAL_BUFFER_SIZE - 1)
        serialCommand += c;
    }
  }
}

void printChannelStatus() {
  Serial.println(F("---- CHANNEL STATUS ----"));
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    Serial.print(F("CH"));
    Serial.print(i);
    Serial.print(F(": State="));
    Serial.print(channelState[i] ? "ON " : "OFF");
    Serial.print(F(", Mode="));
    Serial.print(channelMode[i]);
    Serial.print(F(", LastTrig="));
    Serial.print(lastTrigger[i]);
    Serial.print(F(" ms"));

    if (ignoreInput[i]) {
      Serial.print(F(", Debounce till "));
      Serial.print(ignoreUntil[i]);
    }
    Serial.println();
  }
  Serial.println(F("------------------------"));
}

// ============================= SETUP / LOOP =============================

void setup() {
  Serial.begin(74880);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    bool state = digitalRead(buttonPins[i]);

    btnCurrentState[i] = state;
    btnLastState[i] = state;
    btnLastChangeTime[i] = millis();
  }

  #if USE_RGB_LED
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  led(LOW, R);
  led(LOW, G);
  led(LOW, B);
#else
  pinMode(LED_PIN, OUTPUT);
  led(LOW);
#endif

#if defined(ESP8266) || defined(ESP32)
  EEPROM.begin(MAX_EEPROM_SIZE);
#endif

  uint8_t count = EEPROM.read(0);
  if (count == 0xFF || count > 20) {
    EEPROM.write(0, 0);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();
#endif
    Serial.println(F("EEPROM initialized"));
  }

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    uint8_t val = EEPROM.read(MAX_EEPROM_SIZE - CHANNEL_COUNT + i);
    if (val == 0xFF) {
      writeChannelEEPROM(i, 1, 0);  // режим 1, состояние 0
    }
  }

  initChannels();

  // Вход в режим обучения при удержании кнопки >5 сек при старте
  uint32_t startTime = millis();
  while (millis() - startTime < LONG_PRESS_MS) {
    if (digitalRead(BUTTON_PIN) != LOW) break;
  }
  if (millis() - startTime >= LONG_PRESS_MS) {
    trainingMode();
    initChannels();
  }

  rf.begin();
  dumpEEPROM();
  Serial.println(F("RF Decoder ready"));
  Serial.print(F("Stored codes: "));
  Serial.println(eepromCount());

#if USE_WIFI
  WiFi.mode(WIFI_STA);
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(180);
  mqttClient.setUsernamePassword(mqttUser, mqttPass);
  
  String APName = getChipID();
  APName.toUpperCase();
  APName = "Switch-"+APName;
  if (wm.autoConnect(APName.c_str())) {
    Serial.println("WI-FI connected...");
  } else {
    Serial.println("Configportal running");
  }

  //ArduinoOTA
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.println("Telnet started on port 23");

  if (WiFi.status() == WL_CONNECTED) {
    
    if (!mqttClient.connect(broker, port)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
    } else {
      Serial.println("You're connected to the MQTT broker!");
      mqttClient.onMessage(onMqttMessage);  // устанавливаем обработчик
      mqttClient.subscribe("cmnd/ESP-32test/#");
    }

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
#endif
}

void loop() {

#if USE_WIFI
  wm.process();
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    if (telnetServer.hasClient()) {
      if (telnetClient && telnetClient.connected()) telnetClient.stop();
      telnetClient = telnetServer.available();
      remotePrintln("Telnet connected!");
    }
    // если MQTT отключился → пытаемся переподключиться раз в 5 секунд
    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastMqttReconnectAttempt > 5000) {
        lastMqttReconnectAttempt = now;
        if (!mqttClient.connect(broker, port)) {
          Serial.print("MQTT connection failed! Error code = ");
          Serial.println(mqttClient.connectError());
        } else {
          mqttClient.onMessage(onMqttMessage);  // устанавливаем обработчик
          mqttClient.subscribe("cmnd/ESP-32test/#");
        }
      }
    } else {
      mqttClient.poll();
    }
  } else {
    unsigned long now1 = millis();
    if (now1 - lastWifiReconnectAttempt > 10000) {
      lastWifiReconnectAttempt = now1;
      Serial.println("Wi-Fi lost, reconnecting...");
      WiFi.reconnect();
    }
  }
#endif
  handleChannelButtons();
  handleButton();
  updateOutputs();
  handleSerialCommands();

  if (learningChannel || learningMode) {
    blinkNonBlocking(100);
  } else {
    led(LOW);
  }

  if (rf.isReady()) {
    const uint8_t *data = rf.getBuffer();
    uint8_t size = rf.getSize();
    bool foundFlag = false;
    bool found = false;
    led(HIGH);
    if (!learningChannel) {
      rf.reset();
    }

    uint8_t buf[64];
    uint8_t chan, len, mode;
    uint8_t count = eepromCount();

    for (uint8_t i = 0; i < count; i++) {
      if (!eepromLoadCode(i, mode, chan, buf, len)) continue;
      if (len == size && codesEqual(buf, data, size)) {
        if (learningChannel || learningMode) {
          if (learnChannel == chan) {
            Serial.println(F("ERROR this code matching for this channel"));
            foundFlag = true;
          }
        } else {
          found = true;
          //Serial.print(i);
          //Serial.print(",");
          //Serial.print(chan);
          //Serial.print(",");
          //Serial.println(mode);
          processChannel(chan - 1, mode);
        }
      }
    }

    if (learningChannel) {
      if (foundFlag) {
        Serial.println(F("ERROR channel - this code matching for this channel"));
        rf.reset();
      } else {
        learningChannel = false;
        learningMode = true;
        led(HIGH);
        delay(2000);
        led(LOW);
        Serial.println(F("Wait recieving RF code for channel accept"));
        rf.reset();
      }
    } else if (learningMode) {
      if (foundFlag) {
        Serial.println(F("ERROR mode - this code matching for this channel"));
      } else {
        eepromSaveCode(learnMode, learnChannel, data, size);
        learningMode = false;
        Serial.println(F("Code learned and saved"));
      }
    } else {
      if (!found) {
        Serial.println(F("Unknown code"));
      }
    }
  }

  if (learningChannel && millis() - learnStartTime > LEARN_TIMEOUT) {
    learningChannel = false;
    Serial.println(F("Learning channel timeout"));
  }
  if (learningMode && millis() - learnStartTime > LEARN_TIMEOUT) {
    learningMode = false;
    Serial.println(F("Learning mode timeout"));
  }
}
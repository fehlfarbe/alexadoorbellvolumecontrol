#include <Arduino.h>
#include <AiEsp32RotaryEncoder.h>
#include <WiFi.h>
#include <ESPAsyncWiFiManager.h>
#include <fauxmoESP.h>
#include <ESP32Servo.h>

#define TAG "AlexaBell"
#define DEVICE_NAME "Door bell"

#define PIN_SERVO 16
#define SERVO_MIN 10
#define SERVO_MAX 180
#define SERVO_STEP 3
#define SERVO_DETACH_TIMEOUT_MS 5000

#define ENCODER_A 12
#define ENCODER_B 13
#define ENCODER_BTN 15

// WiFi stuff
WiFiEvent_t wifiState;
AsyncWebServer server(80);
DNSServer dns;
AsyncWiFiManager wifiManager(&server, &dns);
fauxmoESP fauxmo;

// motor control
Servo servo;
unsigned long lastMovement = 0;
uint8_t lastPosition = 0;

// buttons
AiEsp32RotaryEncoder encoder(ENCODER_B, ENCODER_A, ENCODER_BTN, -1, SERVO_STEP, false);

TaskHandle_t pLedTask;
TaskHandle_t pManualControlTask;

bool isVolumeOn();
uint8_t readVolume();
void setState(unsigned char device_id, const char *device_name, bool state, unsigned char value);
void moveTo(int angle);
void manualControlTask(void *parameter);
void WiFiEvent(WiFiEvent_t event);
void ledTask(void *parameter);

void IRAM_ATTR readEncoderISR()
{
  encoder.readEncoder_ISR();
}

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  // setup servo
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50); // standard 50 hz servo
  servo.attach(PIN_SERVO);

  Serial.printf("Init servo from %d to %d\n", SERVO_MIN, SERVO_MAX);
  for (int i = SERVO_MIN; i <= SERVO_MAX; i++)
  {
    Serial.printf("%d -> %d [%d...%d]\n", servo.read(), i, SERVO_MIN, SERVO_MAX);
    servo.write(i);
    delay(10);
  }
  moveTo((SERVO_MIN + SERVO_MAX) / 2);

  // setup encoder for manual control
  encoder.begin();
  encoder.setup(readEncoderISR);
  encoder.disableAcceleration();
  xTaskCreatePinnedToCore(
      manualControlTask,             /* Function to implement the task */
      "manualControlTask",           /* Name of the task */
      getArduinoLoopTaskStackSize(), /* Stack size in words */
      NULL,                          /* Task input parameter */
      1,                             /* Priority of the task */
      &pManualControlTask,           /* Task handle. */
      0);                            /* Core where the task should run */

  // setup WiFi
  pinMode(LED_BUILTIN, OUTPUT);
  xTaskCreatePinnedToCore(
      ledTask,                       /* Function to implement the task */
      "ledTask",                     /* Name of the task */
      getArduinoLoopTaskStackSize(), /* Stack size in words */
      NULL,                          /* Task input parameter */
      1,                             /* Priority of the task */
      &pLedTask,                     /* Task handle. */
      0);                            /* Core where the task should run */
  WiFi.onEvent(WiFiEvent);
  WiFi.setAutoReconnect(true);
  if (!wifiManager.autoConnect("alexa-door-bell"))
  {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
  }
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.broadcastIP());

  // setup alexa device
  fauxmo.addDevice(DEVICE_NAME);
  fauxmo.setState(DEVICE_NAME, false, 0);
  fauxmo.setPort(80); // required for gen3 devices
  fauxmo.onSetState(setState);
  fauxmo.enable(true);
}

void loop()
{
  // update fauxmo
  fauxmo.handle();
}

bool isVolumeOn() {
  return servo.read() == SERVO_MIN;
}

uint8_t readVolume() {
  return map(SERVO_MAX, SERVO_MIN, 0, 255, servo.read());
}

void setState(unsigned char device_id, const char *device_name, bool state, unsigned char value)
{
  Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
  // set position
  if (state)
  {
    moveTo(SERVO_MIN);
  }
  else
  {
    moveTo(SERVO_MAX);
  }

  fauxmo.setState(DEVICE_NAME, isVolumeOn(), readVolume());
}

void moveTo(int angle)
{
  if (!servo.attached())
  {
    Serial.println("Reattach servo");
    servo.attach(PIN_SERVO);
    // servo looses position on detach, so reset to last position before detach
    servo.write(lastPosition);
  }

  Serial.printf("Go to %ddeg\n", angle);
  servo.write(angle);
  lastMovement = millis();
}

void manualControlTask(void *parameter)
{
  while (true)
  {
    // Serial.printf("Current pos %d\n", servo.read());
    auto current = servo.attached() ? servo.read() : lastPosition;

    // handle buttons
    auto enc = encoder.encoderChanged();
    if (enc < 0)
    {
      Serial.println("Volume down");
      moveTo(max(SERVO_MIN, current - SERVO_STEP));
    }
    else if (enc > 0)
    {
      Serial.println("Volume up");
      moveTo(min(SERVO_MAX, current + SERVO_STEP));
    }

    // servo sleep
    if (millis() - lastMovement > SERVO_DETACH_TIMEOUT_MS && servo.attached())
    {
      Serial.println("Detach servo");
      lastMovement = millis();
      lastPosition = servo.read();
      servo.detach();
    }

    delay(25);
  }
}

void WiFiEvent(WiFiEvent_t event)
{
  wifiState = event;
  ESP_LOGI(TAG, "WiFi mode %d", WiFi.getMode());

  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_LOST_IP:
    ESP_LOGI(TAG, "STA Disconnected -> reconnect");
    WiFi.begin();
    break;
  }
}

void ledTask(void *parameter)
{
  uint16_t delayMs = 250;
  while (true)
  {
    switch (wifiState)
    {
    case ARDUINO_EVENT_WIFI_AP_START:
      // ESP_LOGI(TAG, "AP Started");
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      // ESP_LOGI(TAG, "AP user connected");
      break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
      // ESP_LOGI(TAG, "AP Stopped");
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      // ESP_LOGI(TAG, "STA Started");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      // ESP_LOGI(TAG, "STA Connected");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // ESP_LOGI(TAG, "STA IPv4: ");
      // ESP_LOGI(TAG, "%s", WiFi.localIP());
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      // ESP_LOGI(TAG, "STA Disconnected -> reconnect");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      // WiFi.begin();
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      // ESP_LOGI(TAG, "STA Stopped");
      break;
    default:
      break;
    }
    // ESP_LOGI(TAG, "WiFi mode %d", WiFi.getMode());
    delay(delayMs);
  }
}
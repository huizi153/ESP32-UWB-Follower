#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define JDY Serial2

volatile bool isRunning = false;
unsigned long lastDummyTime = 0;
String jdyBuf = "";  // 缓存收到的字符

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!isRunning) return;
  char msg[len + 1];
  memcpy(msg, data, len);
  msg[len] = '\0';
  unsigned int dist; int angle;
  if (sscanf(msg, "Dist:%u,Angle:%d", &dist, &angle) == 2) {
    Serial.printf("%u,%d\n", dist, angle);
  }
}

void handleJDY() {
  while (JDY.available()) {
    char c = JDY.read();
    jdyBuf += c;
    
    if (jdyBuf.length() > 10) jdyBuf = jdyBuf.substring(jdyBuf.length() - 10);  // 防溢出

    if (jdyBuf.indexOf("110") >= 0) {
      isRunning = true;
      JDY.print("START\n");
      jdyBuf = "";
    }
    if (jdyBuf.indexOf("010") >= 0) {
      isRunning = false;
      JDY.print("STOP\n");
      jdyBuf = "";
    }
  }
}

void setup() {
  Serial.begin(115200);
  JDY.begin(9600, SERIAL_8N1, 18, 17);
  delay(100);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) while (1);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  handleJDY();
  if (!isRunning && millis() - lastDummyTime >= 50) {
    Serial.printf("100,0\n");
    lastDummyTime = millis();
  }
  delay(10);
}

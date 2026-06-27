#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define UWB_RX_PIN 4  // 监听 4 号引脚
#define UWB_TX_PIN -1

#define FRAME_LEN        20
#define HEAD0            0x20
#define HEAD1            0x01

uint8_t buf[FRAME_LEN];
uint8_t buf_index = 0;

uint16_t u16_swap(uint16_t data) {
    return ((data & 0xFF) << 8) | ((data >> 8) & 0xFF);
}

// 修复过编译报错的纯净解析逻辑
int parse_uwb_frame(uint8_t *buffer, uint16_t *dist_cm, int16_t *azimuth) {
    if (buffer[0] != HEAD0 || buffer[1] != HEAD1) return -1;
    
    uint16_t dist_raw = (uint16_t)buffer[14] | ((uint16_t)buffer[15] << 8);
    *dist_cm = u16_swap(dist_raw);

    uint16_t azi_raw = (uint16_t)buffer[16] | ((uint16_t)buffer[17] << 8);
    uint16_t azi_swap = u16_swap(azi_raw);
    *azimuth = *(int16_t*)&azi_swap;
    
    return 0;
}

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial1.begin(115200, SERIAL_8N1, UWB_RX_PIN, UWB_TX_PIN);
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // 锁死信道 1
  
  if (esp_now_init() != ESP_OK) return;

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  Serial.println("【纯净发射端】初始化完成，监听 4 号引脚...");
}

void loop() {
  while (Serial1.available()) {
    buf[buf_index++] = Serial1.read();
    if (buf_index >= FRAME_LEN) {
      uint16_t dist; int16_t azi;
      if (parse_uwb_frame(buf, &dist, &azi) == 0) {
        char tx_buffer[32];
        sprintf(tx_buffer, "Dist:%u,Angle:%d", dist, azi);
        Serial.printf("[发送数据] %s\n", tx_buffer); 
        esp_now_send(broadcastAddress, (uint8_t *)tx_buffer, strlen(tx_buffer));
      }
      memmove(buf, buf + 1, FRAME_LEN - 1);
      buf_index = FRAME_LEN - 1;
    }
  }#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  char msg[len + 1];
  memcpy(msg, incomingData, len);
  msg[len] = '\0'; 
  
  unsigned int dist = 0; int angle = 0;
  // 直接提取数字并纯净打印给电脑 Python 脚本
  if (sscanf(msg, "Dist:%u,Angle:%d", &dist, &angle) == 2) {
      Serial.printf("%u,%d\n", dist, angle); 
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // 锁死信道 1
  
  if (esp_now_init() != ESP_OK) while(1); 
  
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() { 
  delay(1000); 
}
}

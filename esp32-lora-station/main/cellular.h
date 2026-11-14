// Đường mạng 4G Cat.1 dự phòng cho trạm bờ (module Ai-Thinker Ca-01, UART2, PPPoS qua esp_modem).
// Hướng dẫn nối dây và kiểm thử: docs/tram-bo-4g.md
#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_netif.h"

// Khởi động module 4G và task tự quay số lại. Gọi SAU esp_netif_init() và esp_event_loop_create_default().
// Có IP qua 4G thì đặt up_bit trong eg, mất IP thì xóa. Không chặn: module chưa trả lời thì task thử lại ở nền.
void cellular_start(EventGroupHandle_t eg, EventBits_t up_bit);

// Đặt DNS công cộng (8.8.8.8, 1.1.1.1). DNS của lwIP dùng chung cho mọi card mạng: DNS của hotspot WiFi
// (192.168.x.1) không dùng được khi đi bằng 4G, còn DNS nhà mạng thường không trả lời khi đi bằng WiFi.
void net_use_public_dns(esp_netif_t *netif);

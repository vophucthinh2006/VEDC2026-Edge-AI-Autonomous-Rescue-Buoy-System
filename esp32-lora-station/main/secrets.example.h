// Sao chép file này thành main/secrets.h rồi điền giá trị thật. secrets.h đã nằm trong .gitignore.
#pragma once

#define WIFI_SSID          "ten-wifi-hoac-hotspot"
#define WIFI_PASSWORD      "mat-khau-wifi"

// Địa chỉ Worker sau khi deploy (xem cloud/README hoặc README.md), luôn là https
#define CLOUD_INGEST_URL   "https://mliot-redgescue.<tai-khoan>.workers.dev/api/ingest"
// Phải trùng secret INGEST_TOKEN đặt bằng: npx wrangler secret put INGEST_TOKEN
#define CLOUD_INGEST_TOKEN "chuoi-bi-mat-dai-ngau-nhien"

// Đường lên cloud (docs/tram-bo-4g.md). Mặc định bật cả hai: còn WiFi thì đi WiFi, mất thì tự sang 4G.
// Thử riêng 4G: đặt NET_USE_WIFI 0.
#define NET_USE_WIFI       1
#define NET_USE_CELL       1

// APN của SIM trong module 4G Ca-01: VinaPhone "m3-world", Viettel "v-internet", MobiFone "m-wap"
#define CELL_APN           "m3-world"

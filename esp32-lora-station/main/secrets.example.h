// Sao chép file này thành main/secrets.h rồi điền giá trị thật. secrets.h đã nằm trong .gitignore.
#pragma once

#define WIFI_SSID          "ten-wifi-hoac-hotspot"
#define WIFI_PASSWORD      "mat-khau-wifi"

// Địa chỉ Worker sau khi deploy (xem cloud/README hoặc README.md), luôn là https
#define CLOUD_INGEST_URL   "https://mliot-redgescue.<tai-khoan>.workers.dev/api/ingest"
// Phải trùng secret INGEST_TOKEN đặt bằng: npx wrangler secret put INGEST_TOKEN
#define CLOUD_INGEST_TOKEN "chuoi-bi-mat-dai-ngau-nhien"

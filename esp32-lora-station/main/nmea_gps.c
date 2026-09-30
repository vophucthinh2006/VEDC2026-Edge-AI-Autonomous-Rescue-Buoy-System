#include "nmea_gps.h"
#include <string.h>
#include <stdlib.h>

static char line_buf[128];
static int line_len = 0;
static unsigned long so_byte_da_nhan = 0;
static unsigned long so_cau_gga_da_parse = 0;

static double nmea_to_decimal(double raw, char dir) {
    int deg = (int)(raw / 100);
    double min = raw - deg * 100.0;
    double dec = deg + min / 60.0;
    if (dir == 'S' || dir == 'W') dec = -dec;
    return dec;
}

// Tách chuỗi NMEA theo dấu phẩy, trả về mảng con trỏ tới từng trường
// (ghi đè ký tự ',' thành '\0' ngay trên buffer - không cấp phát thêm bộ nhớ)
static int tach_truong(char *s, char *fields[], int max_fields) {
    int n = 0;
    char *p = s;
    fields[n++] = p;
    while (*p && n < max_fields) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[n++] = p + 1;
        }
        p++;
    }
    return n;
}

static bool parse_gga(char *sentence, gps_fix_t *fix) {
    char *f[16] = {0};
    int n = tach_truong(sentence, f, 16);
    if (n < 10) return false;

    // f[0]="$GPGGA", f[1]=time, f[2]=lat, f[3]=N/S, f[4]=lon, f[5]=E/W,
    // f[6]=fix quality, f[7]=so ve tinh, f[8]=HDOP
    int fix_quality = atoi(f[6]);
    if (fix_quality <= 0 || strlen(f[2]) == 0 || strlen(f[4]) == 0) {
        fix->fix_valid = false;
        return true; // parse duoc cau GGA nhung chua co fix
    }

    double raw_lat = atof(f[2]);
    double raw_lon = atof(f[4]);
    fix->lat = nmea_to_decimal(raw_lat, f[3][0]);
    fix->lon = nmea_to_decimal(raw_lon, f[5][0]);
    fix->satellites = atoi(f[7]);
    fix->hdop = (float)atof(f[8]);
    fix->fix_valid = true;
    return true;
}

bool nmea_feed_char(char c, gps_fix_t *fix) {
    so_byte_da_nhan++;
    if (c == '\r') return false;

    if (c == '\n') {
        bool ket_qua = false;
        if (line_len > 6) {
            line_buf[line_len] = '\0';
            // Chấp nhận cả $GPGGA (chỉ GPS) và $GNGGA (đa hệ thống vệ tinh)
            if (strncmp(line_buf, "$GPGGA", 6) == 0 || strncmp(line_buf, "$GNGGA", 6) == 0) {
                ket_qua = parse_gga(line_buf, fix);
                if (ket_qua) so_cau_gga_da_parse++;
            }
        }
        line_len = 0;
        return ket_qua;
    }

    if (line_len < (int)sizeof(line_buf) - 1) {
        line_buf[line_len++] = c;
    } else {
        // dong qua dai / loi - reset de tranh tran bo dem
        line_len = 0;
    }
    return false;
}

unsigned long nmea_debug_so_byte_da_nhan(void) {
    return so_byte_da_nhan;
}

unsigned long nmea_debug_so_cau_gga_da_parse(void) {
    return so_cau_gga_da_parse;
}

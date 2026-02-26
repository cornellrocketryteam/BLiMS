// i2c_scan.cpp — scans i2c0 and i2c1 on common pin pairs
// Build same way as controller_test, just swap the .cpp filename

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "tusb.h"
#include <cstdio>

struct PinPair { uint sda; uint scl; };

void scan_bus(i2c_inst_t *bus, const char *name, uint sda, uint scl) {
    // Deinit first in case it was already configured
    i2c_deinit(bus);
    
    i2c_init(bus, 100 * 1000);  // 100kHz for reliability
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    
    printf("# Scanning %s on SDA=%d SCL=%d:\n", name, sda, scl);
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        int ret = i2c_read_blocking(bus, addr, &dummy, 1, false);
        if (ret >= 0) {
            printf("#   0x%02X", addr);
            if (addr == 0x42) printf(" <- u-blox GPS");
            else if (addr == 0x48) printf(" <- ADS1015");
            else if (addr == 0x77) printf(" <- BMP390");
            else if (addr == 0x76) printf(" <- BMP alt addr");
            else if (addr == 0x29) printf(" <- LIS3DH alt addr");
            else if (addr == 0x28) printf(" <- BNO055");
            printf("\n");
            found++;
        }
    }
    if (found == 0) printf("#   (nothing found)\n");
    
    // Reset pins back to default
    gpio_set_function(sda, GPIO_FUNC_NULL);
    gpio_set_function(scl, GPIO_FUNC_NULL);
    i2c_deinit(bus);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    while (!tud_cdc_connected()) {
        sleep_ms(500);
    }
    
    printf("# ========================================\n");
    printf("# I2C Bus Scanner - both buses, all pins\n");
    printf("# ========================================\n");
    
    // i2c0 valid pin pairs
    PinPair i2c0_pairs[] = {
        {0, 1}, {4, 5}, {8, 9}, {12, 13}, {16, 17}, {20, 21}
    };
    
    // i2c1 valid pin pairs
    PinPair i2c1_pairs[] = {
        {2, 3}, {6, 7}, {10, 11}, {14, 15}, {18, 19}, {26, 27}
    };
    
    for (auto &p : i2c0_pairs) {
        scan_bus(i2c0, "i2c0", p.sda, p.scl);
    }
    
    for (auto &p : i2c1_pairs) {
        scan_bus(i2c1, "i2c1", p.sda, p.scl);
    }
    
    printf("# ========================================\n");
    printf("# Scan complete. Look for 0x42 (GPS).\n");
    printf("# ========================================\n");
    
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}
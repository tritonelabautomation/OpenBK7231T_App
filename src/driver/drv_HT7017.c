#include "../new_common.h"
#include "drv_ht7017.h"
#include "../obk_config.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_uart.h"

#if ENABLE_DRIVER_HT7017

// Constants
#define HT7017_UART_RECEIVE_BUFFER_SIZE 256
#define REG_RMS_I   0x06
#define REG_RMS_U   0x08
#define REG_POWER_P 0x0A

static unsigned short ht7017_baudRate = 4800;

typedef struct {
    int32_t voltage_raw;
    int32_t current_raw;
    int32_t power_raw;
} ht7017_data_t;

static ht7017_data_t g_ht7017 = {0};
static uint8_t last_requested_reg = 0;

// Helper for 24-bit signed conversion
static int32_t Int24ToInt32(int32_t val) {
    return (val & (1 << 23) ? val | (0xFF << 24) : val);
}

static void ScaleAndUpdate(ht7017_data_t *data) {
    float voltage = data->voltage_raw * 0.001f;
    float current = data->current_raw * 0.0001f;
    float power   = Int24ToInt32(data->power_raw) * 0.01f;
    
    BL_ProcessUpdate(voltage, current, power, 0.0f, 0.0f);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: V=%.2fV I=%.4fA P=%.2fW", 
              voltage, current, power);
}

static int HT7017_UART_TryToGetNextPacket() {
    uint8_t buf[6];
    int available = UART_GetDataSize();
    
    // Show exactly what we have
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: Attempting parse with %d bytes available", available);
    
    if(available < 1) {
        return 0;
    }
    
    // Dump whatever bytes we have (up to 6)
    int bytes_to_show = (available > 6) ? 6 : available;
    if(bytes_to_show >= 1) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Available bytes: %02X %02X %02X %02X %02X %02X (%d total)",
                  bytes_to_show >= 1 ? UART_GetByte(0) : 0xFF,
                  bytes_to_show >= 2 ? UART_GetByte(1) : 0xFF,
                  bytes_to_show >= 3 ? UART_GetByte(2) : 0xFF,
                  bytes_to_show >= 4 ? UART_GetByte(3) : 0xFF,
                  bytes_to_show >= 5 ? UART_GetByte(4) : 0xFF,
                  bytes_to_show >= 6 ? UART_GetByte(5) : 0xFF,
                  available);
    }
    
    // If we have at least 4 bytes, try to parse
    if(available >= 4) {
        for(int i=0; i<4; i++) {
            buf[i] = UART_GetByte(i);
        }
        
        uint8_t status = buf[0];
        int32_t raw_val = (buf[1] << 16) | (buf[2] << 8) | buf[3];
        
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Parsed 4-byte - Status=0x%02X Raw=0x%06X (%d) for reg 0x%02X", 
                  status, raw_val, raw_val, last_requested_reg);
        
        switch(last_requested_reg) {
            case REG_RMS_U:   g_ht7017.voltage_raw = raw_val; break;
            case REG_RMS_I:   g_ht7017.current_raw = raw_val; break;
            case REG_POWER_P: g_ht7017.power_raw   = raw_val; break;
        }
        
        ScaleAndUpdate(&g_ht7017);
        UART_ConsumeBytes(4);
        return 4;
    }
    
    // Otherwise just consume the junk byte
    if(available >= 1) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Only %d byte(s), discarding 0x%02X", 
                  available, UART_GetByte(0));
        UART_ConsumeBytes(1);
        return 1;
    }
    
    return 0;
}

void HT7017_Init(void) {
    delay_ms(50);
    
    UART_InitUART(ht7017_baudRate, 0, false);
    UART_InitReceiveRingBuffer(HT7017_UART_RECEIVE_BUFFER_SIZE);
    
    delay_ms(100);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: Sending write enable sequence");
    
    // Try writing enable sequence differently - maybe it needs ACK?
    UART_SendByte(0x6A);
    UART_SendByte(0x52);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x32);
    UART_SendByte(0xEE);
    
    delay_ms(100); // Give more time
    
    // Clear whatever came back
    int startup_bytes = UART_GetDataSize();
    if(startup_bytes > 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Startup response: %d bytes", startup_bytes);
        // Show first few bytes
        for(int i=0; i<startup_bytes && i<10; i++) {
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                      "HT7017: Startup byte[%d] = 0x%02X", i, UART_GetByte(i));
        }
        UART_ConsumeBytes(startup_bytes);
    }
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017 Initialized at %d baud", ht7017_baudRate);
}

void HT7017_RunEverySecond(void) {
    static int read_step = 0;
    static int request_count = 0;
    uint8_t target_reg;
    int avail = UART_GetDataSize();
    
    request_count++;
    
    if(avail > 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "HT7017: RX bytes=%i", avail);
    }
    
    // Try to parse
    int parsed = 0;
    while(parsed < 10 && UART_GetDataSize() >= 1) {
        int result = HT7017_UART_TryToGetNextPacket();
        if(result == 0) {
            break;
        }
        parsed++;
    }
    
    // Clear overflow
    avail = UART_GetDataSize();
    if(avail > 50) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Buffer overflow (%d bytes), clearing", avail);
        UART_ConsumeBytes(avail);
    }
    
    // Every 10th request, try the write enable again
    if(request_count % 10 == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Re-sending write enable (attempt %d)", request_count/10);
        UART_SendByte(0x6A);
        UART_SendByte(0x52);
        UART_SendByte(0x00);
        UART_SendByte(0x00);
        UART_SendByte(0x32);
        UART_SendByte(0xEE);
        delay_ms(10);
    }
    
    // Send next read request
    if (read_step == 0)      target_reg = REG_RMS_U;
    else if (read_step == 1) target_reg = REG_RMS_I;
    else                     target_reg = REG_POWER_P;
    
    last_requested_reg = target_reg;
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: Sending [6A %02X]", target_reg);
    
    UART_SendByte(0x6A);
    UART_SendByte(target_reg);
    
    read_step = (read_step + 1) % 3;
}

#endif

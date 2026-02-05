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
    // Scaling factors
    float voltage = data->voltage_raw * 0.001f;
    float current = data->current_raw * 0.0001f;
    float power   = Int24ToInt32(data->power_raw) * 0.01f;
    
    BL_ProcessUpdate(voltage, current, power, 0.0f, 0.0f);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: V=%.2fV I=%.4fA P=%.2fW", 
              voltage, current, power);
}

// Parse 4-byte response packet
static int HT7017_UART_TryToGetNextPacket() {
    uint8_t buf[4];
    int available = UART_GetDataSize();
    
    if(available < 4) {
        return 0;
    }
    
    // Read 4 bytes
    for(int i=0; i<4; i++) {
        buf[i] = UART_GetByte(i);
    }
    
    // ALWAYS dump raw bytes (no debug counter limit)
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: RAW [%02X %02X %02X %02X] for reg 0x%02X",
              buf[0], buf[1], buf[2], buf[3], last_requested_reg);
    
    // Parse: First byte could be status, next 3 are data
    uint8_t status = buf[0];
    int32_t raw_val = (buf[1] << 16) | (buf[2] << 8) | buf[3];
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: Status=0x%02X Raw=0x%06X (%d)", 
              status, raw_val, raw_val);
    
    // Assign based on last request
    switch(last_requested_reg) {
        case REG_RMS_U:   
            g_ht7017.voltage_raw = raw_val; 
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "HT7017: Updated voltage");
            break;
        case REG_RMS_I:   
            g_ht7017.current_raw = raw_val;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "HT7017: Updated current");
            break;
        case REG_POWER_P: 
            g_ht7017.power_raw   = raw_val;
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "HT7017: Updated power");
            break;
        default:
            addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                      "HT7017: Unknown register 0x%02X", last_requested_reg);
            break;
    }
    
    ScaleAndUpdate(&g_ht7017);
    
    UART_ConsumeBytes(4);
    return 4;
}

void HT7017_Init(void) {
    delay_ms(50);
    
    UART_InitUART(ht7017_baudRate, 0, false);
    UART_InitReceiveRingBuffer(HT7017_UART_RECEIVE_BUFFER_SIZE);
    
    delay_ms(100);
    
    // Disable write protection
    UART_SendByte(0x6A);
    UART_SendByte(0x52);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(0x32);
    UART_SendByte(0xEE);
    
    delay_ms(50);
    
    // Clear startup data
    int initial_bytes = UART_GetDataSize();
    if(initial_bytes > 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Clearing %d startup bytes", initial_bytes);
        UART_ConsumeBytes(initial_bytes);
    }
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017 Initialized at %d baud", ht7017_baudRate);
}

void HT7017_RunEverySecond(void) {
    static int read_step = 0;
    uint8_t target_reg;
    int avail = UART_GetDataSize();
    
    if(avail > 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, "HT7017: RX bytes=%i", avail);
    }
    
    // Try to parse all available packets
    int parsed = 0;
    while(parsed < 10 && UART_GetDataSize() >= 4) {
        int result = HT7017_UART_TryToGetNextPacket();
        if(result == 0) {
            break;
        }
        parsed++;
    }
    
    // Clear overflow
    avail = UART_GetDataSize();
    if(avail > 100) {
        addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
                  "HT7017: Buffer overflow, clearing %d bytes", avail);
        UART_ConsumeBytes(avail);
    }
    
    // Send next read request
    if (read_step == 0)      target_reg = REG_RMS_U;
    else if (read_step == 1) target_reg = REG_RMS_I;
    else                     target_reg = REG_POWER_P;
    
    last_requested_reg = target_reg;
    
    // Send 2-byte read request
    UART_SendByte(0x6A);
    UART_SendByte(target_reg);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017: Sent read request for reg 0x%02X", target_reg);
    
    read_step = (read_step + 1) % 3;
}

#endif

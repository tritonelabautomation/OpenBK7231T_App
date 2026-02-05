#include "../new_common.h"
#include "drv_ht7017.h"
#include "../obk_config.h"
#include "../logging/logging.h"
#include "../new_cfg.h"
#include "../new_pins.h"
#include "drv_uart.h"

#if ENABLE_DRIVER_HT7017

#define HT7017_UART_RECEIVE_BUFFER_SIZE 256

// Register addresses
#define REG_RMS_U    0x08  // Voltage
#define REG_RMS_I    0x06  // Current
#define REG_POWER_P  0x0A  // Active Power
#define REG_FREQ     0x09  // Frequency
#define REG_WRITE_ENABLE 0x52
#define WRITE_ENABLE_KEY 0x32

// Calibration coefficients (similar to BL0942 approach)
// These can be adjusted via commands: ht7017_cal_voltage, ht7017_cal_current, ht7017_cal_power
static float g_voltage_cal = 8.185f;   // Raw to Volts multiplier
static float g_current_cal = 0.045f;   // Raw to Amps multiplier  
static float g_power_cal = 1.0f;       // Power adjustment factor
static int g_noise_threshold = 14;     // Values <= this are considered noise

static unsigned short ht7017_baudRate = 4800;

typedef struct {
    int32_t voltage_raw;
    int32_t current_raw;
    int32_t freq_raw;
    int32_t power_raw;
} ht7017_data_t;

static ht7017_data_t g_ht7017 = {0};
static uint8_t last_requested_reg = 0;

// BL0942-style calibration functions
static commandResult_t HT7017_Calibrate_Voltage(const void *context, const char *cmd, const char *args, int flags) {
    float new_cal;
    
    if(args == 0 || *args == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HT7017: Current voltage calibration = %.3f", g_voltage_cal);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "Usage: ht7017_cal_voltage [multiplier]");
        return CMD_RES_OK;
    }
    
    new_cal = atof(args);
    if(new_cal <= 0 || new_cal > 100) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "HT7017: Invalid calibration value (must be 0-100)");
        return CMD_RES_BAD_ARGUMENT;
    }
    
    g_voltage_cal = new_cal;
    addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HT7017: Voltage calibration set to %.3f", g_voltage_cal);
    return CMD_RES_OK;
}

static commandResult_t HT7017_Calibrate_Current(const void *context, const char *cmd, const char *args, int flags) {
    float new_cal;
    
    if(args == 0 || *args == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HT7017: Current current calibration = %.3f", g_current_cal);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "Usage: ht7017_cal_current [multiplier]");
        return CMD_RES_OK;
    }
    
    new_cal = atof(args);
    if(new_cal <= 0 || new_cal > 10) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "HT7017: Invalid calibration value (must be 0-10)");
        return CMD_RES_BAD_ARGUMENT;
    }
    
    g_current_cal = new_cal;
    addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HT7017: Current calibration set to %.3f", g_current_cal);
    return CMD_RES_OK;
}

static commandResult_t HT7017_Calibrate_Power(const void *context, const char *cmd, const char *args, int flags) {
    float new_cal;
    
    if(args == 0 || *args == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HT7017: Current power calibration = %.3f", g_power_cal);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "Usage: ht7017_cal_power [multiplier]");
        return CMD_RES_OK;
    }
    
    new_cal = atof(args);
    if(new_cal <= 0 || new_cal > 10) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "HT7017: Invalid calibration value (must be 0-10)");
        return CMD_RES_BAD_ARGUMENT;
    }
    
    g_power_cal = new_cal;
    addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "HT7017: Power calibration set to %.3f", g_power_cal);
    return CMD_RES_OK;
}

static void ScaleAndUpdate(ht7017_data_t *data) {
    // Apply noise threshold
    float voltage_raw = (data->voltage_raw <= g_noise_threshold) ? 0 : data->voltage_raw;
    float current_raw = (data->current_raw <= g_noise_threshold) ? 0 : data->current_raw;
    
    // Apply calibration
    float voltage = voltage_raw * g_voltage_cal;
    float current = current_raw * g_current_cal;
    
    // Power can be calculated or use raw value
    float power;
    if(data->power_raw > g_noise_threshold) {
        // Use HT7017's power reading if available
        power = data->power_raw * 0.01f * g_power_cal;
    } else {
        // Calculate from V×I
        power = voltage * current * g_power_cal;
    }
    
    // Frequency calculation (HT7017 specific)
    float freq = 0.0f;
    if(data->freq_raw >= 25 && data->freq_raw <= 35) {
        freq = 50.0f;  // 50Hz for raw value ~27
    } else if(data->freq_raw >= 50 && data->freq_raw <= 70) {
        freq = 60.0f;  // 60Hz
    }
    
    // Update OpenBK channels
    BL_ProcessUpdate(voltage, current, power, freq, 0.0f);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
        "HT7017: V=%.1fV I=%.3fA P=%.1fW F=%.1fHz (Raw: V=%d I=%d P=%d F=%d)",
        voltage, current, power, freq, 
        data->voltage_raw, data->current_raw, data->power_raw, data->freq_raw);
}

static void HT7017_SendWriteEnable(void) {
    UART_SendByte(0x6A);
    UART_SendByte(REG_WRITE_ENABLE);
    UART_SendByte(0x00);
    UART_SendByte(0x00);
    UART_SendByte(WRITE_ENABLE_KEY);
    // Checksum: (0x6A + 0x52 + 0x00 + 0x00 + 0x32) & 0xFF = 0xEE
    UART_SendByte(0xEE);
    delay_ms(10);
}

static int HT7017_UART_TryToGetNextPacket() {
    uint8_t buf[5];
    int available = UART_GetDataSize();
    
    if(available < 4) return 0;
    
    // Read 4 bytes (status + 24-bit data)
    for(int i = 0; i < 4; i++) {
        buf[i] = UART_GetByte(i);
    }
    
    // Parse data
    int32_t raw_val = (buf[1] << 16) | (buf[2] << 8) | buf[3];
    
    // Assign to appropriate register
    switch(last_requested_reg) {
        case REG_RMS_U:   g_ht7017.voltage_raw = raw_val; break;
        case REG_RMS_I:   g_ht7017.current_raw = raw_val; break;
        case REG_FREQ:    g_ht7017.freq_raw    = raw_val; break;
        case REG_POWER_P: g_ht7017.power_raw   = raw_val; break;
    }
    
    ScaleAndUpdate(&g_ht7017);
    
    // Consume the 4 data bytes
    UART_ConsumeBytes(4);
    
    // Check for trailer byte
    if(UART_GetDataSize() >= 1) {
        uint8_t trailer = UART_GetByte(0);
        if(trailer != 0x6A) {  // Not start of next packet
            UART_ConsumeBytes(1);
        }
    }
    
    return 4;
}

void HT7017_Init(void) {
    delay_ms(50);
    
    UART_InitUART(ht7017_baudRate, 0, false);
    UART_InitReceiveRingBuffer(HT7017_UART_RECEIVE_BUFFER_SIZE);
    
    delay_ms(100);
    
    // Send initial write enable
    HT7017_SendWriteEnable();
    delay_ms(50);
    
    // Clear startup data
    int startup_bytes = UART_GetDataSize();
    if(startup_bytes > 0) {
        UART_ConsumeBytes(startup_bytes);
    }
    
    // Register calibration commands (BL0942 style)
    CMD_RegisterCommand("ht7017_cal_voltage", HT7017_Calibrate_Voltage, NULL);
    CMD_RegisterCommand("ht7017_cal_current", HT7017_Calibrate_Current, NULL);
    CMD_RegisterCommand("ht7017_cal_power", HT7017_Calibrate_Power, NULL);
    
    addLogAdv(LOG_INFO, LOG_FEATURE_ENERGYMETER, 
              "HT7017 Initialized at %d baud (V=%.3f, I=%.3f, P=%.3f)", 
              ht7017_baudRate, g_voltage_cal, g_current_cal, g_power_cal);
}

void HT7017_RunEverySecond(void) {
    static unsigned int read_counter = 0;
    static int read_step = 0;
    uint8_t target_reg;
    
    read_counter++;
    
    // Read every 5 seconds (gives HT7017 time to settle)
    if(read_counter < 5) return;
    read_counter = 0;
    
    // Process any waiting data first
    while(UART_GetDataSize() >= 4) {
        if(HT7017_UART_TryToGetNextPacket() == 0) break;
    }
    
    // Clear any junk/partial data
    int avail = UART_GetDataSize();
    if(avail > 0 && avail < 4) {
        UART_ConsumeBytes(avail);
    }
    
    // Cycle through registers
    switch(read_step) {
        case 0: target_reg = REG_RMS_U;   break;  // Voltage
        case 1: target_reg = REG_RMS_I;   break;  // Current
        case 2: target_reg = REG_FREQ;    break;  // Frequency
        case 3: target_reg = REG_POWER_P; break;  // Power
        default: target_reg = REG_RMS_U; read_step = 0; break;
    }
    
    last_requested_reg = target_reg;
    
    // Send write enable before each read (HT7017 auto-locks after reads)
    HT7017_SendWriteEnable();
    delay_ms(50);
    
    // Send read request
    UART_SendByte(0x6A);
    UART_SendByte(target_reg);
    
    delay_ms(100);  // Wait for response
    
    read_step = (read_step + 1) % 4;
}

void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request)
{
    hprintf255(request, "<h5>HT7017 Calibration:</h5>");
    hprintf255(request, "Voltage: %.3f, Current: %.3f, Power: %.3f<br>", 
               g_voltage_cal, g_current_cal, g_power_cal);
    hprintf255(request, "Commands: ht7017_cal_voltage, ht7017_cal_current, ht7017_cal_power<br>");
}

#endif

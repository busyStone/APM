#ifndef __AP_Z60_ANALOG_SOURCE_H__
#define __AP_Z60_ANALOG_SOURCE_H__

#include <AP_HAL.h>

/* Predefined telemetry responses. */
#define TELEMETRY_RESP_OK         "_OK_"
#define TELEMETRY_RESP_FAIL       "FAIL"

/* Telemetry start-of-frame signature. */
#define TELEMETRY_MSG_SOF         0xBD
/* Empty message ID. */
#define TELEMETRY_MSG_NOMSG       0x00
/* Telemetry buffer size in bytes.  */
#define TELEMETRY_BUFFER_SIZE     0x80
/* Telemetry message header size in bytes.  */
#define TELEMETRY_MSG_HDR_SIZE    0x04
/* Telemetry message checksum size in bytes.  */
#define TELEMETRY_MSG_CRC_SIZE    0x04
/* Telemetry message header + crc size in bytes.  */
#define TELEMETRY_MSG_SVC_SIZE    ( TELEMETRY_MSG_HDR_SIZE + TELEMETRY_MSG_CRC_SIZE )


typedef struct tagTelemetryMessage {
    uint8_t sof;
    uint8_t msg_id;
    uint8_t size;
    uint8_t res;
    char data[TELEMETRY_BUFFER_SIZE];
    uint32_t crc;
} TelemetryMessage;

class AP_Z60_AnalogSource : public AP_HAL::AnalogSource
{
public:
    AP_Z60_AnalogSource( AP_HAL::UARTDriver *port );
    float           read_average(void);
    float           read_latest(void);
    void            set_pin(uint8_t);
    float	    voltage_average();
    float	    voltage_latest() { return voltage_average(); }
    float	    voltage_average_ratiometric() { return voltage_average(); }
    float       read_current_amps();
    float       read_current_total_mah();
    uint8_t     read_remains_pct();

    // stop pins not implemented on ADC yet
    void set_stop_pin(uint8_t p) {}
    void set_settle_time(uint16_t settle_time_ms) {}
    void run() { updateRequest(); update(); }
private:
    uint32_t crc32(uint32_t pBuf[], size_t length);
    uint32_t getCRC32Checksum(TelemetryMessage *pMsg);
    void readSerialDataResync(uint8_t len);
    void processMessage();
    void sendUpdateRequestMsg(uint8_t msg_id);
    void update();
    void updateRequest();
    size_t read(uint8_t *buf, uint32_t len);
        
    AP_HAL::UARTDriver *_port;
    TelemetryMessage _msg;
    uint8_t *_msgPos;
    size_t _bytesRequired;
    uint16_t _voltage;
    int16_t  _current_amps;
    uint16_t _current_remaining_mah;
    uint16_t _percentage;
};

#endif

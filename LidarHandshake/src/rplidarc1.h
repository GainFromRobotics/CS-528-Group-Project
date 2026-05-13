#include <HardwareSerial.h>

// commands
#define REQ_STOP 0x25
#define REQ_RESET 0x40
#define REQ_SCAN 0x20
#define REQ_EXPRESS_SCAN 0x82
#define REQ_GET_INFO 0x50
#define REQ_GET_HEALTH 0x52
#define REQ_GET_SAMPLERATE 0x59
#define REQ_GET_LIDAR_CONF 0x84

// byte macros
#define START_FLAG1 0xA5 // also known as the sync byte
#define START_FLAG2 0x5A // some commands have 2 sync bytes instead of 1
#define SEND_MODE_SINGLE_RESPONSE 0x0
#define SEND_MODE_MULTIPLE_RESPONSE 0x1

// default values
#define BAUD_RATE_DEFAULT 460800
#define UART_DEFAULT 2
#define TIMEOUT_MS_DEFAULT 500

// error and success codes
#define LIDAR_SUCCESS 0
#define LIDAR_ERR_GENERIC -1
#define LIDAR_ERR_TIMEOUT -2
#define LIDAR_ERR_BUFFER_TOO_SMALL -3

// =============================================================
// Data Response Structures
// =============================================================

// Response Desciptor Structure
typedef struct _response_descriptor {
    uint8_t start_flag1;
    uint8_t start_flag2;
    size_t data_len;
    uint8_t send_mode; // Currently two options exist: single mode and multi mode
    uint8_t data_type;
} response_descriptor;
#define RESPONSE_DESCRIPTOR_PACKET_LENGTH 7
const response_descriptor rd_empty = {0,0,0,0,0};

// Device Info Structure
typedef struct _device_info {
    uint8_t major_model;
    uint8_t sub_model;
    uint8_t firmware_minor;
    uint8_t firmware_major;
    uint8_t hardware;
    char serial_number[17];
} device_info;
#define DEVICE_INFO_PACKET_LENGTH 20
const device_info di_empty = {0,0,0,0,0};

// Device Health Structure
typedef struct _device_health {
    uint8_t status;
    uint16_t err_code;
} device_health;
#define DEVICE_HEALTH_PACKET_LENGTH 3
const device_health dh_empty = {0,0};

// Device Samplerate Structure
typedef struct _device_sample_rate {
    uint16_t Tstandard;
    uint16_t Texpress;
} device_sample_rate;
#define DEVICE_SAMPLE_RATE_PACKET_LENGTH 4
const device_sample_rate dsr_empty = {0,0};

// =============================================================
// Scan Data Response Structures
// =============================================================

// Traditional Scan Structure
typedef struct _scan_sample_traditional {
    bool start_flag;
    uint8_t quality;
    float angle; //degrees
    float distance; //mm
} scan_standard_sample;
#define SCAN_STANDARD_PACKET_LENGTH 5
const scan_standard_sample sss_empty = {false,0,0.0f,0.0f};

// Express Scan Structure
#define CABIN_COUNT 40
typedef struct _scan_sample_express {
    bool start_flag;
    float angles[CABIN_COUNT]; //degrees
    float distances[CABIN_COUNT]; //mm

    uint8_t checksum;
} scan_express_sample;
#define SCAN_EXPRESS_PACKET_LENGTH 84
const scan_express_sample ses_empty = {}; //it's just completely empty

// =============================================================
// RPLidar C1 Class For Arduino Framework
// =============================================================

// None of this code has any safety mechanisms in place and is left entirely up to a "trust me bro" until I fix that!

// NONE of this code is tested! When we get the chance, do some trial runs to see that this does in fact operate as expected!
class RPLidarC1 {
    private:
        HardwareSerial lidarSerial;

        uint8_t rx_pin;
        uint8_t tx_pin;
        uint8_t uart_number;
        long baudrate;

        // UART error flags
        bool uart_break_error = false;
        bool uart_buffer_full_error = false;
        bool uart_fifo_ovf_error = false;
        bool uart_frame_error = false;
        bool uart_parity_error = false;
        bool uart_unknown_error = false;

    public:
        RPLidarC1(uint8_t uart_num, uint8_t rx, uint8_t tx, long baud);
        RPLidarC1(uint8_t uart_num, uint8_t rx, uint8_t tx);
        RPLidarC1(uint8_t rx, uint8_t tx);

        void onUartError(hardwareSerial_error_t err);
        // begin
        void begin();

        // Error flag getters
        bool hasUartBreakError() const { return uart_break_error; }
        bool hasUartBufferFullError() const { return uart_buffer_full_error; }
        bool hasUartFifoOvfError() const { return uart_fifo_ovf_error; }
        bool hasUartFrameError() const { return uart_frame_error; }
        bool hasUartParityError() const { return uart_parity_error; }
        bool hasUartUnknownError() const { return uart_unknown_error; }
        
        // Check if any UART errors occurred
        bool hasAnyUartError() const {
            return uart_break_error || uart_buffer_full_error || uart_fifo_ovf_error ||
                   uart_frame_error || uart_parity_error || uart_unknown_error;
        }
        
        // Print which error flags are set
        void printUartErrors() const;
        
        void clearUartErrors() {
            uart_break_error = false;
            uart_buffer_full_error = false;
            uart_fifo_ovf_error = false;
            uart_frame_error = false;
            uart_parity_error = false;
            uart_unknown_error = false;
        }

        //Send a command
        int send_request_packet(uint8_t cmd, uint8_t * payload, size_t pl_n);

        // Handles getting data
        int get_n_bytes(uint8_t * buf, size_t b_n, unsigned long TIMEOUT_MS);
        int get_n_bytes(uint8_t * buf, size_t b_n);

        // Get our response descriptors
        response_descriptor create_response_descriptor(uint8_t * response_packet_raw, unsigned int len);
        response_descriptor get_response_descriptor(unsigned long TIMEOUT_MS);
        response_descriptor get_response_descriptor();

        // Generic data response retreiver we can use later
        int get_data_response(response_descriptor rd, uint8_t * buf, size_t b_n, unsigned long TIMEOUT_MS);
        int get_data_response(response_descriptor rd, uint8_t * buf, size_t b_n);

        // Commands
        void cmd_stop(); // no response
        void cmd_reset(); // no response
        
        // traditional scan
        response_descriptor cmd_scan(); //Just send the scan command
        scan_standard_sample get_latest_scan(response_descriptor rd); // get latest scan

        // express scan
        response_descriptor cmd_express_scan(uint8_t * payload, unsigned int pl_n); // Just send the scan command
        response_descriptor cmd_express_scan();
        scan_express_sample get_latest_express_scan(response_descriptor rd); // get latest scan

        device_info cmd_get_info(); // single response
        device_health cmd_get_health(); // single response
        device_sample_rate cmd_get_samplerate(); // single response

        void cmd_get_lidar_conf(uint8_t * payload, unsigned int pl_n); // oh GOD what the hell are you

};

 // get strings
char * to_string(response_descriptor rd, char* buf, unsigned int n);
char * to_string(device_info di, char * buf, unsigned int n);
char * to_string(device_health dh, char * buf, unsigned int n);
char * to_string(device_sample_rate dsr, char * buf, unsigned int n);

char * to_string(scan_standard_sample sss, char * buf, unsigned int n);
char * to_string(scan_express_sample ses, char * buf, unsigned int n);
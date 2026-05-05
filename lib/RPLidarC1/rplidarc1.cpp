#include "rplidarc1.h"

#include <string.h>
#include <stdio.h>

// =============================================================
// RPLidar C1 Class For Arduino Framework
// =============================================================

// RPLidarC1 constructor
// uart_num: Which UART channel is the RPLidarC1 attached to?
// rx: which pin is our rx pin? (should be a gpio pin)
// tx: which pin irs our tx pin? (should be a gpio pin)
// baud: baud rate of lidar
RPLidarC1::RPLidarC1(uint8_t uart_num, uint8_t rx, uint8_t tx, long baud) : lidarSerial(uart_num) {
    uart_number = uart_num;
    rx_pin = rx;
    tx_pin = tx;
    baudrate = baud;
    printf("uart:%d,rx:%d,tx:%d,baud:%d\n",uart_num,rx_pin,tx_pin,baud);
}

RPLidarC1::RPLidarC1(uint8_t uart_num, uint8_t rx, uint8_t tx) : lidarSerial(uart_num) {
    RPLidarC1(uart_num, rx, tx, BAUD_RATE_DEFAULT);
}

RPLidarC1::RPLidarC1(uint8_t rx, uint8_t tx) : lidarSerial(UART_DEFAULT) {
    RPLidarC1(UART_DEFAULT, rx, tx, BAUD_RATE_DEFAULT);
}

// Sends a request packet with a payload that we can send to the lidar
// cmd: the command byte
// payload: custom payload. This will depend onthe command
// pl_n: payload length
// this function returns the size of the generated request_packet. If it's negative it's an error
int RPLidarC1::send_request_packet(uint8_t cmd, uint8_t * payload, size_t pl_n) {

    // Special case where we have no payload
    if(pl_n == 0) {
        uint8_t request_packet[] = {START_FLAG1,cmd};
        this->lidarSerial.write(request_packet,2);
        return 2;
    }

    size_t packet_size = 4 + pl_n; // 1 for sync byte, 1 for command, 1 for payload size, pl_n bytes for payload, 1 for checksum

    uint8_t c = 0 ^ START_FLAG1 ^ cmd; // checksum byte
    uint8_t request_packet[packet_size]; // the request_packet
    request_packet[0] = START_FLAG1;
    request_packet[1] = cmd;
    request_packet[2] = pl_n;
    for(int i = 0; i < pl_n; i++) {
        request_packet[i+3] = payload[i];
        c ^= payload[i];
    }

    // Write the request packet
    this->lidarSerial.write(request_packet,packet_size);

    return packet_size;
}

void RPLidarC1::begin() {
    printf("uart:%d,rx:%d,tx:%d,baud:%d\n",uart_number,rx_pin,tx_pin,baudrate);
    lidarSerial.begin(baudrate,SERIAL_8N1,rx_pin,tx_pin);
}


//#define READ_BYTES_DEBUG

// Generic UART byte reader that we can use later
// buf: buffer to store information
// b_n: sie of buffer
// TIMEOUT_MS: timeout before we give up on reading our data
// we return how many bytes we were able to read
int RPLidarC1::get_n_bytes(uint8_t * buf, size_t b_n, unsigned long TIMEOUT_MS) {

#ifdef READ_BYTES_DEBUG
    printf("[bytes to read]: %d\n",b_n);
#endif
    int i = 0;
    while(i < b_n) {
        unsigned long start_ms = millis();
        while(millis()-start_ms < TIMEOUT_MS) {
            if(lidarSerial.available()) {
                buf[i] = lidarSerial.read();
#ifdef READ_BYTES_DEBUG
                printf("buf[%d]:%02x\n",i,buf[i]);
#endif
                // we got our byte? awesome get the next one
                goto next_byte;
            }
            //printf("No lidar read at %lu < %lu\n",millis()-start_ms,TIMEOUT_MS);
        }
        printf("we timed out! we read %d our of %d bytes\n",i,b_n);
        return i; // If we're here it means we timed out. Return how many bytes we WERE able to get
        next_byte:
        i++;
    }
    return i;
}

int RPLidarC1::get_n_bytes(uint8_t * buf, size_t b_n) {
    return get_n_bytes(buf,b_n,TIMEOUT_MS_DEFAULT);
}

// response: the data that we will transform into a response descriptor
// len: the size of the data
#define SEND_MODE_BYTE_MASK 0b11000000
response_descriptor RPLidarC1::create_response_descriptor(uint8_t * response, unsigned int len) {

    if(len < RESPONSE_DESCRIPTOR_PACKET_LENGTH) {
        return rd_empty; 
    }

    response_descriptor rd_ret;

    rd_ret.start_flag1 = response[0];
    rd_ret.start_flag2 = response[1];

    uint8_t d[] = {response[2],response[3],response[4],response[5]};

    uint32_t data_len = 0; // first 30 bits
    uint8_t send_mode = 0; // last 2 bits

    // extract data length
    data_len |= d[0];
    data_len |= d[1] << 8;
    data_len |= d[2] << 16;
    data_len |= (d[3]&~SEND_MODE_BYTE_MASK) << 24;

    // extract send mode
    send_mode = d[3]&SEND_MODE_BYTE_MASK >> 6;

    rd_ret.data_len = data_len;
    rd_ret.send_mode = send_mode;

    rd_ret.data_type = response[6];

    return rd_ret;
}

// If there's an error, we return an empty response descriptor
response_descriptor RPLidarC1::get_response_descriptor(unsigned long TIMEOUT_MS) {
    // Create buffer
    uint8_t response_descriptor_raw[RESPONSE_DESCRIPTOR_PACKET_LENGTH];

    // Get response descriptor
    int n = get_n_bytes(response_descriptor_raw,RESPONSE_DESCRIPTOR_PACKET_LENGTH,TIMEOUT_MS);
    //printf("we got %d bytes for our response descriptor",n);

    // validate response descriptor
    if(n != RESPONSE_DESCRIPTOR_PACKET_LENGTH) {
        printf("Failed to read every byte, %d != %d (descriptor packet length)\n",n,RESPONSE_DESCRIPTOR_PACKET_LENGTH);
        return rd_empty;
    }

    // return the processed data
    return create_response_descriptor(response_descriptor_raw,RESPONSE_DESCRIPTOR_PACKET_LENGTH);
}

response_descriptor RPLidarC1::get_response_descriptor() {
    return get_response_descriptor(TIMEOUT_MS_DEFAULT);
}

// A generic data response retreiver that we can call later
// rd: The response descriptor. What type of data are we getting?
// buf: The buffer to store our retreived data
// b_n: the size of our buffer
// TIMEOUT_MS: How long should we wait before giving up on our data?
// Returns how many bytes were read. If the return value is negative, it's an error code
int RPLidarC1::get_data_response(response_descriptor rd, uint8_t * buf, size_t b_n, unsigned long TIMEOUT_MS) {
    uint8_t send_mode = rd.send_mode; // relevant maybe

    size_t data_response_size = rd.data_len;

    // Check if our data buffer is long enough
    if(b_n < data_response_size) {
        return LIDAR_ERR_BUFFER_TOO_SMALL;
    }

    int n = get_n_bytes(buf,b_n,TIMEOUT_MS);

    // validate received data
    if(n < rd.data_len) {
        return LIDAR_ERR_TIMEOUT;
    }

    return n;
}

int RPLidarC1::get_data_response(response_descriptor rd, uint8_t * buf, size_t b_n) {
    return get_data_response(rd,buf,b_n,TIMEOUT_MS_DEFAULT);
}

// wait at least 10ms before sending another request
void RPLidarC1::cmd_stop() {
    uint8_t request_packet[] = {START_FLAG1,REQ_STOP};
    lidarSerial.write(request_packet,2);
}

// wait at least 500ms before sending another request
void RPLidarC1::cmd_reset() {
    uint8_t request_packet[] = {START_FLAG1,REQ_RESET};
    lidarSerial.write(request_packet,2);
}

response_descriptor RPLidarC1::cmd_scan() {
    uint8_t request_packet[] = {START_FLAG1,REQ_SCAN};
    lidarSerial.write(request_packet,2);

    delay(200);

    // get response descriptor
    response_descriptor rd = get_response_descriptor();

    char tb[256];
    printf("scan rd: %s\n",to_string(rd,tb,256));

    // Check that we have the correct response desciptor
    if(rd.data_len != SCAN_STANDARD_PACKET_LENGTH) {
        return rd_empty;
    }

    return rd;
}

// Get the latest standard scan
#define START_FLAG_MASK     0b00000001
#define INV_START_FLAG_MASK 0b00000010
#define QUALITY_MASK        0b11111100
#define CHECK_BIT_MASK      0b00000001
#define ANGLE_Q6_MASK       0b11111110
scan_standard_sample RPLidarC1::get_latest_scan(response_descriptor rd) {

    //verify our response descriptor
    if(rd.data_len != SCAN_STANDARD_PACKET_LENGTH) {
        printf("%d != %d\n",rd.data_len, SCAN_STANDARD_PACKET_LENGTH);
        return sss_empty;
    }

    // get our data
    const uint32_t data_len = rd.data_len;
    uint8_t data[data_len];
    int status = get_data_response(rd,data,data_len);

    // Error handler
    if(status < 0) {
        printf("Failed to read scan data\n");
        return sss_empty;
    }

    // extract our scan data
    // check start bit

    bool start_bit = (data[0] & START_FLAG_MASK) == START_FLAG_MASK;
    bool inv_start_bit = (data[0] & INV_START_FLAG_MASK) == INV_START_FLAG_MASK;
    bool check_bit = (data[1] & CHECK_BIT_MASK) == CHECK_BIT_MASK;

    //printf("S:%d,!S:%d,C:%d\n",start_bit,inv_start_bit,check_bit);
    
    // If the start bit and the inv start bit aren't inverses, then it's not a valid sample
    // if the check bit is 0, then it's not a valid sample
    if((start_bit == inv_start_bit) || (!check_bit)) {
        //printf("start_bit %d, inv_start_bit %d\n",start_bit,inv_start_bit);
        //printf("check_bit %d\n", check_bit);
        //return sss_empty;
        inv_start_bit = !start_bit; //dumb solution but whatever
    }

    scan_standard_sample scan;

    scan.start_flag = start_bit;
    scan.quality = (data[0] & QUALITY_MASK) >> 2;

    // get angle
    unsigned short angle_q6 = (data[1] & ANGLE_Q6_MASK) >> 1;
    angle_q6 |= (data[2] << 7);
    //angle_q6 = angle_q6 >> 1;

    unsigned short distance_q2 = data[3];
    distance_q2 |= (data[4] << 8);

    // https://en.wikipedia.org/wiki/Q_(number_format)
    scan.angle = ((float)angle_q6)/64.0f;
    scan.distance = ((float)distance_q2)/4.0f;

    return scan;
}

// tell the lidar to start express scanning
response_descriptor RPLidarC1::cmd_express_scan(uint8_t * payload, unsigned int pl_n) {
    
    // Make request packet
    int status = send_request_packet(REQ_EXPRESS_SCAN,payload,pl_n);

    // Error handler
    if(status < 0) {
        return rd_empty;
    }

    // Get response descriptor
    response_descriptor rd = get_response_descriptor();

    // Check that we have the correct response desciptor
    if(rd.data_len != SCAN_EXPRESS_PACKET_LENGTH) {
        return rd_empty;
    }

    return rd;
}

response_descriptor RPLidarC1::cmd_express_scan() {
    uint8_t pl[] = {0x1,0x0,0x0,0x0,0x0};
    return cmd_express_scan(pl,5);
}

// helper function
float angle_difference(float a, float b) {

    float a_norm = fmodf(a,360.0f);
    float b_norm = fmodf(b,360.0f);
    float diff = a_norm-b_norm;

    float d1 = abs(diff);
    float d2 = 360.0f-d1;
    float dir = (diff >= 0) ? 1 : -1; // 1 means clockwise, -1 means counter clock wise

    return dir*fminf(d1,d2);
}

// get the latest scan
#define BYTE_L_MASK 0b00001111
#define BYTE_U_MASK 0b11110000
#define E_START_FLAG_MASK 0b10000000
scan_express_sample RPLidarC1::get_latest_express_scan(response_descriptor rd) {

    // Verify our response descriptor
    if(rd.data_len != SCAN_EXPRESS_PACKET_LENGTH) {
        return ses_empty;
    }

    // get our data
    const uint32_t data_len = rd.data_len;
    uint8_t data[data_len];
    int status = get_data_response(rd,data,data_len); // change this to also get the next angle...

    // Error handler
    if(status < 0) {
        return ses_empty;
    }

    // process our data
    static float current_angle = 0.0; // This will change from scan to scan...
    scan_express_sample ses_ret;

    uint8_t sync_byte = (data[0] & BYTE_U_MASK) >> 4;
    sync_byte |= (data[1] & BYTE_U_MASK);

    uint8_t checksum_byte = (data[0] & BYTE_L_MASK) >> 4;
    checksum_byte |= (data[1] & BYTE_L_MASK);

    uint16_t start_angle_q6 = data[2];
    start_angle_q6 |= (data[3] & ~E_START_FLAG_MASK) << 8;
    float start_angle = ((float)start_angle_q6)/64.0f;

    bool is_start = (data[3] & E_START_FLAG_MASK == E_START_FLAG_MASK);

    for(int i = 0; i < CABIN_COUNT; i++) {
        uint16_t cabin = data[i+4];
        cabin |= (data[i+4] << 4);

        // oh GOD computing the angle is gonna SUCK
        // I need to know the NEXT angle received to compute the angle difference.... FUCK
        ses_ret.distances[i] = cabin;
    }

    return ses_ret;
}

// helper function
unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

#define MAJOR_MODEL_MASK 0b11110000
#define MINOR_MODEL_MASK ~MAJOR_MODEL_MASK
device_info RPLidarC1::cmd_get_info() {
    uint8_t request_packet[] = {START_FLAG1,REQ_GET_INFO};
    lidarSerial.write(request_packet,2);
    
    delay(200);
    // get response descriptor
    response_descriptor rd = get_response_descriptor();

    char buf2[256];
    printf("Response Descriptor = (%s)\n",to_string(rd,buf2,256));

    // confirm that our response descriptor is correct
    if(rd.data_len != DEVICE_INFO_PACKET_LENGTH) {
        printf("obtained invalid response descriptor! (probably empty)\n");
        return di_empty;
    }

    printf("waiting for data...\n");
    // get our data
    uint8_t data[rd.data_len];
    int status = get_data_response(rd,data,rd.data_len);

    // didn't work? womp womp
    if(status < 0) {
        printf("Failed to read our data response\n");
        return di_empty;
    }
    printf("Got our data!\n");

    // process and return our data
    device_info di_ret;

    di_ret.major_model = (data[0] & MAJOR_MODEL_MASK) >> 4;
    di_ret.sub_model = data[0] & MINOR_MODEL_MASK;
    di_ret.firmware_minor = data[1];
    di_ret.firmware_major = data[2];
    di_ret.hardware = data[3];
    for(int i = 0; i < 16; i++) {
        uint8_t num = data[4+i];
        di_ret.serial_number[i] = num;
    }
    di_ret.serial_number[16] = 0; //null terminator

    return di_ret;
}

device_health RPLidarC1::cmd_get_health() {
    uint8_t request_packet[] = {START_FLAG1,REQ_GET_HEALTH};
    lidarSerial.write(request_packet,2);

    delay(200);

    // get response descriptor
    response_descriptor rd = get_response_descriptor();

    // confirm that our response descriptor is correct
    if(rd.data_len != DEVICE_HEALTH_PACKET_LENGTH) {
        return dh_empty;
    }

    // get our data
    uint8_t data[rd.data_len];
    int status = get_data_response(rd,data,rd.data_len);

    // didn't work? womp womp
    if(status < 0) {
        return dh_empty;
    }

    // process and return our data
    device_health dh_ret;

    dh_ret.status = data[0];
    dh_ret.err_code = data[1];
    dh_ret.err_code |= data[2] << 8;

    return dh_ret;
}

device_sample_rate RPLidarC1::cmd_get_samplerate() {
    uint8_t request_packet[] = {START_FLAG1,REQ_GET_SAMPLERATE};
    lidarSerial.write(request_packet,2);

    delay(200);

    // get response descriptor
    response_descriptor rd = get_response_descriptor();
    
    // confirm that our response descriptor is correct
    if(rd.data_len != DEVICE_SAMPLE_RATE_PACKET_LENGTH) {
        return dsr_empty;
    }

    // get our data
    uint8_t data[rd.data_len];
    int status = get_data_response(rd,data,rd.data_len);

    // didn't work? womp womp
    if(status < 0) {
        return dsr_empty;
    }

    // Process and return our data
    device_sample_rate dsr_ret;
    dsr_ret.Tstandard = data[0];
    dsr_ret.Tstandard |= data[1] << 8;
    dsr_ret.Texpress = data[2];
    dsr_ret.Texpress |= data[3] << 8;

    return dsr_ret;
}

void RPLidarC1::cmd_get_lidar_conf(uint8_t * payload, unsigned int len) {

}

char * to_string(response_descriptor rd, char* buf, unsigned int n) {
    if (n == 0) return buf;
    
    snprintf(buf, n,
             "start_flag1=%x start_flag2=%x data_len=%lu send_mode=%u data_type=%u",
             rd.start_flag1,
             rd.start_flag2,
             (unsigned long)rd.data_len,   // %lu for uint32_t
             rd.send_mode,
             rd.data_type);
    
    return buf;
}

char * to_string(device_info di, char * buf, unsigned int n) {
    if (n == 0) return buf;
    
    //printf("huh\n");
    snprintf(buf, n,
             "major_model=%u sub_model=%u firmware_minor=%u firmware_major=%u hardware=%u serial_number=%s",
             di.major_model,
             di.sub_model,
             di.firmware_minor,
             di.firmware_major,
             di.hardware,
             di.serial_number);
    
    //printf("oh ok\n");
    return buf;
}

char * to_string(device_health dh, char * buf, unsigned int n) {
    if (n == 0) return buf;
    
    snprintf(buf, n,
             "status=%u err_code=%u",
             dh.status,
             dh.err_code);
    
    return buf;
}

char * to_string(device_sample_rate dsr, char * buf, unsigned int n) {
    if (n == 0) return buf;
    
    snprintf(buf, n,
             "Tstandard=%u Texpress=%u",
             dsr.Tstandard,
             dsr.Texpress);
    
    return buf;
}

char * to_string(scan_standard_sample sss, char * buf, unsigned int n) {
    if (n == 0) return buf;

    snprintf(buf, n,
            "start: %d,quality: %u, angle: %.3f degrees, dist: %.3f mm",
            sss.start_flag,
            sss.quality,
            sss.angle,
            sss.distance
    );

    return buf;
}
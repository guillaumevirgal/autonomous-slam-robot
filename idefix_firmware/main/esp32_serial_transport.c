/* 
esp32_serial_transport.c

micro-ROS custom UART transport for the ESP32-S3. Implements the four uxrCustomTransport callbacks: open, close, write, read.

Modified  upstream file (examples/int32_publisher_custom_transport in the vendored micro_ros_espidf_component) 
 */

#include "esp32_serial_transport.h"
#include "pins.h"

#include <uxr/client/transport.h>

#include "driver/uart.h"
#include "driver/gpio.h"

// UART ring buffer size on the receive side. Upstream default.
#define UART_BUFFER_SIZE 512 // Increase to 1024 or 2048 if seeing buffer overrun errors



bool esp32_serial_open(struct uxrCustomTransport * transport){
    size_t * uart_port = (size_t *) transport->args;    // dereference the value at the UART port adress we passed in main.c 

    // Configure the UART parameters
    uart_config_t uart_config = {0};                    // zero-init to avoid stale fields
    uart_config.baud_rate = UROS_UART_BAUDRATE;         // from pins.h
    uart_config.data_bits = UART_DATA_8_BITS;           // 8 data bits, no parity, 1 stop bit (8N1)
    uart_config.parity    = UART_PARITY_DISABLE;        
    uart_config.stop_bits = UART_STOP_BITS_1;           
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;   // no RTS/CTS: not wired on our UART0 path

    if (uart_param_config(*uart_port, &uart_config) == ESP_FAIL) {
        return false;                                   // baud/format setup failed
    }
    // Assign the TX and RX pins explicitly, no flow control (RTS or CTS)
    if (uart_set_pin(*uart_port, UROS_UART_TX_GPIO, UROS_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_FAIL) {
        return false;                                   // pin routing failed
    }
    // Install the UART driver with 1024 RX ring buffer, no TX buffer, no event queue and default ISR flags
    if (uart_driver_install(*uart_port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) == ESP_FAIL) {
        return false;                                   // driver install failed
    }

    return true; // transport open, ready for read/write
}


bool esp32_serial_close(struct uxrCustomTransport * transport){
    size_t * uart_port = (size_t *) transport->args;    // dereference the value at the UART port adress we passed in main.c
    return uart_driver_delete(*uart_port) == ESP_OK;    // tear down the driver
}


size_t esp32_serial_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err){
    (void) err;                                         // upstream ignores err too
    size_t * uart_port = (size_t *) transport->args;    // dereference the value at the UART port adress we passed in main.c 
    const int txBytes = uart_write_bytes(*uart_port, (const char *) buf, len);  // blocking write, returns bytes queued
    return (size_t) txBytes;                            // caller inspects short writes if needed
}


size_t esp32_serial_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err){
    (void) err;                                         // upstream ignores err too
    size_t * uart_port = (size_t *) transport->args;    // dereference the value at the UART port adress we passed in main.c
    // timeout is in milliseconds per the micro-XRCE-DDS API
    const int rxBytes = uart_read_bytes(*uart_port, buf, len, timeout / portTICK_PERIOD_MS); // convert to FreeRTOS ticks
    return (size_t) rxBytes;                            // may be 0 on timeout, that is normal
}
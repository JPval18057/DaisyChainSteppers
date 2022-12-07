/*
 * Author:
 * Description:
 * This is for the ESP32
 * A very good tutorial on how to use SPI on the ESP32
 * https://microcontrollerslab.com/esp32-spi-communication-tutorial-arduino/
 * https://randomnerdtutorials.com/esp32-spi-communication-arduino/#spi-multiple-bus-hspi-vspi
 */
//Libraries
#include <ESP32DMASPIMaster.h>

/*
 * Default VSPI pins
 * MOSI Pin: 23
 * MISO Pin: 19
 * SCK Pin: 18
 * SS Pin: 5
 * Default HSPI pins
 * MOSI Pin: 13
 * MISO Pin: 12
 * SCK Pin: 14
 * SS Pin: 15
 */
/* ##################################################################
 * SPI CONFIGURATION
 * ##################################################################
 */
//Pin definitions
//MASTER OUT SLAVE IN
#define MOSI 23
//MASTER IN SLAVE OUT
#define MISO 19
//CHIP SELECT
#define SS 5
//CLOCK FOR DATA TRANSFER
#define CLK 18
//Builtin LED for ESP32
#define led 2

ESP32DMASPI::Master master;

static const uint32_t BUFFER_SIZE = 4; //buffer size of 1 byte because of atmega328p
uint8_t* spi_master_tx_buf;
uint8_t* spi_master_rx_buf;

void set_buffer() {
    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        spi_master_tx_buf[i] = i & 0xFF;
    }
    memset(spi_master_rx_buf, 0, BUFFER_SIZE);
}

//##################################################################

//Functions
void calibrate();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.print("\nDevice started, wait 2s\n");
  delay(2000);
  calibrate();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void calibrate(){
  pinMode(led, OUTPUT);
  pinMode(SS,OUTPUT);
  // to use DMA buffer, use these methods to allocate buffer
  spi_master_tx_buf = master.allocDMABuffer(BUFFER_SIZE);
  spi_master_rx_buf = master.allocDMABuffer(BUFFER_SIZE);
  
  set_buffer();
  delay(5000); //WAIT FOR SOME REASON
  
  master.setDataMode(SPI_MODE0);           // default: SPI_MODE0
  master.setFrequency(1000000);            // default: 8MHz (too fast for bread board...)
  master.setMaxTransferSize(BUFFER_SIZE);  // default: 4092 bytes

  // begin() after setting
  master.begin(VSPI);  // default: HSPI (CS: 15, CLK: 14, MOSI: 13, MISO: 12)

  //TRANSFER THE CALIBRATION DATA
  // start and wait to complete transaction  
  master.transfer((uint8_t*)'A', spi_master_rx_buf, BUFFER_SIZE);
  master.transfer((uint8_t*)'A', spi_master_rx_buf, BUFFER_SIZE);
  master.transfer((uint8_t*)'A', spi_master_rx_buf, BUFFER_SIZE);

  // show received data (if needed)
  for (size_t i = 0; i < BUFFER_SIZE; ++i) {
      printf("Incoming data: %d ", spi_master_rx_buf[i]);
  }
  printf("\n");

  
}

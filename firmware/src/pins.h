#pragma once

// Pin map for the LILYGO T-Beam Supreme (ESP32-S3 + SX1262 + AXP2101 + GNSS).
// Values taken from LilyGO's LoRa-Series utilities.h (T_BEAM_S3_SUPREME_SX1262
// block) in ../Lora Source — this is board-specific, not derivable from a
// generic ESP32-S3 datasheet.

// Display / general I2C bus
#define I2C_SDA         17
#define I2C_SCL         18

// PMU (AXP2101 power management IC) lives on a second I2C bus
#define PMU_SDA         42
#define PMU_SCL         41
#define PMU_IRQ_PIN     40

// GNSS module UART + enable line
#define GPS_RX_PIN      9   // ESP32 RX <- GPS TX
#define GPS_TX_PIN      8   // ESP32 TX -> GPS RX
#define GPS_EN_PIN      7   // must be driven HIGH to power the GNSS chip
#define GPS_PPS_PIN     6
#define GPS_BAUD_RATE   9600

// SOS button
#define BUTTON_PIN      0

// SX1262 LoRa radio (SPI)
#define RADIO_SCLK_PIN  12
#define RADIO_MISO_PIN  13
#define RADIO_MOSI_PIN  11
#define RADIO_CS_PIN    10
#define RADIO_RST_PIN   5
#define RADIO_DIO1_PIN  1
#define RADIO_BUSY_PIN  4

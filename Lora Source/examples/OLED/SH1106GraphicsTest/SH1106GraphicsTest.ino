/*

  GraphicsTest.ino

  Universal 8bit Graphics Library (https://github.com/olikraus/disp/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <Arduino.h>
#include "LoRaBoards.h"


void u8g2_prepare(void)
{
    disp->setFont(u8g2_font_6x10_tf);
    disp->setFontRefHeightExtendedText();
    disp->setDrawColor(1);
    disp->setFontPosTop();
    disp->setFontDirection(0);
}

void u8g2_box_frame(uint8_t a)
{
    disp->drawStr( 0, 0, "drawBox");
    disp->drawBox(5, 10, 20, 10);
    disp->drawBox(10 + a, 15, 30, 7);
    disp->drawStr( 0, 30, "drawFrame");
    disp->drawFrame(5, 10 + 30, 20, 10);
    disp->drawFrame(10 + a, 15 + 30, 30, 7);
}

void u8g2_disc_circle(uint8_t a)
{
    disp->drawStr( 0, 0, "drawDisc");
    disp->drawDisc(10, 18, 9);
    disp->drawDisc(24 + a, 16, 7);
    disp->drawStr( 0, 30, "drawCircle");
    disp->drawCircle(10, 18 + 30, 9);
    disp->drawCircle(24 + a, 16 + 30, 7);
}

void u8g2_r_frame(uint8_t a)
{
    disp->drawStr( 0, 0, "drawRFrame/Box");
    disp->drawRFrame(5, 10, 40, 30, a + 1);
    disp->drawRBox(50, 10, 25, 40, a + 1);
}

void u8g2_string(uint8_t a)
{
    disp->setFontDirection(0);
    disp->drawStr(30 + a, 31, " 0");
    disp->setFontDirection(1);
    disp->drawStr(30, 31 + a, " 90");
    disp->setFontDirection(2);
    disp->drawStr(30 - a, 31, " 180");
    disp->setFontDirection(3);
    disp->drawStr(30, 31 - a, " 270");
}

void u8g2_line(uint8_t a)
{
    disp->drawStr( 0, 0, "drawLine");
    disp->drawLine(7 + a, 10, 40, 55);
    disp->drawLine(7 + a * 2, 10, 60, 55);
    disp->drawLine(7 + a * 3, 10, 80, 55);
    disp->drawLine(7 + a * 4, 10, 100, 55);
}

void u8g2_triangle(uint8_t a)
{
    uint16_t offset = a;
    disp->drawStr( 0, 0, "drawTriangle");
    disp->drawTriangle(14, 7, 45, 30, 10, 40);
    disp->drawTriangle(14 + offset, 7 - offset, 45 + offset, 30 - offset, 57 + offset, 10 - offset);
    disp->drawTriangle(57 + offset * 2, 10, 45 + offset * 2, 30, 86 + offset * 2, 53);
    disp->drawTriangle(10 + offset, 40 + offset, 45 + offset, 30 + offset, 86 + offset, 53 + offset);
}

void u8g2_ascii_1()
{
    char s[2] = " ";
    uint8_t x, y;
    disp->drawStr( 0, 0, "ASCII page 1");
    for ( y = 0; y < 6; y++ ) {
        for ( x = 0; x < 16; x++ ) {
            s[0] = y * 16 + x + 32;
            disp->drawStr(x * 7, y * 10 + 10, s);
        }
    }
}

void u8g2_ascii_2()
{
    char s[2] = " ";
    uint8_t x, y;
    disp->drawStr( 0, 0, "ASCII page 2");
    for ( y = 0; y < 6; y++ ) {
        for ( x = 0; x < 16; x++ ) {
            s[0] = y * 16 + x + 160;
            disp->drawStr(x * 7, y * 10 + 10, s);
        }
    }
}

void u8g2_extra_page(uint8_t a)
{
    disp->drawStr( 0, 0, "Unicode");
    disp->setFont(u8g2_font_unifont_t_symbols);
    disp->setFontPosTop();
    disp->drawUTF8(0, 24, "☀ ☁");
    switch (a) {
    case 0:
    case 1:
    case 2:
    case 3:
        disp->drawUTF8(a * 3, 36, "☂");
        break;
    case 4:
    case 5:
    case 6:
    case 7:
        disp->drawUTF8(a * 3, 36, "☔");
        break;
    }
}

#define cross_width 24
#define cross_height 24
static const unsigned char cross_bits[] U8X8_PROGMEM  = {
    0x00, 0x18, 0x00, 0x00, 0x24, 0x00, 0x00, 0x24, 0x00, 0x00, 0x42, 0x00,
    0x00, 0x42, 0x00, 0x00, 0x42, 0x00, 0x00, 0x81, 0x00, 0x00, 0x81, 0x00,
    0xC0, 0x00, 0x03, 0x38, 0x3C, 0x1C, 0x06, 0x42, 0x60, 0x01, 0x42, 0x80,
    0x01, 0x42, 0x80, 0x06, 0x42, 0x60, 0x38, 0x3C, 0x1C, 0xC0, 0x00, 0x03,
    0x00, 0x81, 0x00, 0x00, 0x81, 0x00, 0x00, 0x42, 0x00, 0x00, 0x42, 0x00,
    0x00, 0x42, 0x00, 0x00, 0x24, 0x00, 0x00, 0x24, 0x00, 0x00, 0x18, 0x00,
};

#define cross_fill_width 24
#define cross_fill_height 24
static const unsigned char cross_fill_bits[] U8X8_PROGMEM  = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x18, 0x64, 0x00, 0x26,
    0x84, 0x00, 0x21, 0x08, 0x81, 0x10, 0x08, 0x42, 0x10, 0x10, 0x3C, 0x08,
    0x20, 0x00, 0x04, 0x40, 0x00, 0x02, 0x80, 0x00, 0x01, 0x80, 0x18, 0x01,
    0x80, 0x18, 0x01, 0x80, 0x00, 0x01, 0x40, 0x00, 0x02, 0x20, 0x00, 0x04,
    0x10, 0x3C, 0x08, 0x08, 0x42, 0x10, 0x08, 0x81, 0x10, 0x84, 0x00, 0x21,
    0x64, 0x00, 0x26, 0x18, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#define cross_block_width 14
#define cross_block_height 14
static const unsigned char cross_block_bits[] U8X8_PROGMEM  = {
    0xFF, 0x3F, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20,
    0xC1, 0x20, 0xC1, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20, 0x01, 0x20,
    0x01, 0x20, 0xFF, 0x3F,
};

void u8g2_bitmap_overlay(uint8_t a)
{
    uint8_t frame_size = 28;

    disp->drawStr(0, 0, "Bitmap overlay");

    disp->drawStr(0, frame_size + 12, "Solid / transparent");
    disp->setBitmapMode(false /* solid */);
    disp->drawFrame(0, 10, frame_size, frame_size);
    disp->drawXBMP(2, 12, cross_width, cross_height, cross_bits);
    if (a & 4)
        disp->drawXBMP(7, 17, cross_block_width, cross_block_height, cross_block_bits);

    disp->setBitmapMode(true /* transparent*/);
    disp->drawFrame(frame_size + 5, 10, frame_size, frame_size);
    disp->drawXBMP(frame_size + 7, 12, cross_width, cross_height, cross_bits);
    if (a & 4)
        disp->drawXBMP(frame_size + 12, 17, cross_block_width, cross_block_height, cross_block_bits);
}

void u8g2_bitmap_modes(uint8_t transparent)
{
    const uint8_t frame_size = 24;

    disp->drawBox(0, frame_size * 0.5, frame_size * 5, frame_size);
    disp->drawStr(frame_size * 0.5, 50, "Black");
    disp->drawStr(frame_size * 2, 50, "White");
    disp->drawStr(frame_size * 3.5, 50, "XOR");

    if (!transparent) {
        disp->setBitmapMode(false /* solid */);
        disp->drawStr(0, 0, "Solid bitmap");
    } else {
        disp->setBitmapMode(true /* transparent*/);
        disp->drawStr(0, 0, "Transparent bitmap");
    }
    disp->setDrawColor(0);// Black
    disp->drawXBMP(frame_size * 0.5, 24, cross_width, cross_height, cross_bits);
    disp->setDrawColor(1); // White
    disp->drawXBMP(frame_size * 2, 24, cross_width, cross_height, cross_bits);
    disp->setDrawColor(2); // XOR
    disp->drawXBMP(frame_size * 3.5, 24, cross_width, cross_height, cross_bits);
}

uint8_t draw_state = 0;

void draw(void)
{
    u8g2_prepare();
    switch (draw_state >> 3) {
    case 0: u8g2_box_frame(draw_state & 7); break;
    case 1: u8g2_disc_circle(draw_state & 7); break;
    case 2: u8g2_r_frame(draw_state & 7); break;
    case 3: u8g2_string(draw_state & 7); break;
    case 4: u8g2_line(draw_state & 7); break;
    case 5: u8g2_triangle(draw_state & 7); break;
    case 6: u8g2_ascii_1(); break;
    case 7: u8g2_ascii_2(); break;
    case 8: u8g2_extra_page(draw_state & 7); break;
    case 9: u8g2_bitmap_modes(0); break;
    case 10: u8g2_bitmap_modes(1); break;
    case 11: u8g2_bitmap_overlay(draw_state & 7); break;
    }
}


void setup(void)
{
    setupBoards();
    if (!disp) {
        Serial.println("No find SH1106 display!Please check whether the connection is normal");
        while (1);
    }
}

void loop(void)
{
    // picture loop
    disp->clearBuffer();
    draw();
    disp->sendBuffer();

    // increase the state
    draw_state++;
    if ( draw_state >= 12 * 8 )
        draw_state = 0;

    // deley between each page
    delay(100);

}

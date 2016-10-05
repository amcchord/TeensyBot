/*
 *
 * new Font
 *
 * created with GLCDFontCreator
 * original framework by F. Maximilian Thiele
 * Modified By Siddharth Kaul
 *
 *
 * File Name           : roosewood_std_size24
 * Date                : 10.11.2012
 * Font size in bytes  : 28312
 * Font width          : 10
 * Font height         : 26
 * Font first char     : 32
 * Font last char      : 128
 * Font used chars     : 96
 *
 * The font data are defined as
 *
 * struct _FONT_ {
 *     uint16_t   font_Size_in_Bytes_over_all_included_Size_it_self;
 *     uint8_t    font_Width_in_Pixel_for_fixed_drawing;
 *     uint8_t    font_Height_in_Pixel_for_all_characters;
 *     unit8_t    font_First_Char;
 *     uint8_t    font_Char_Count;
 *
 *     uint8_t    font_Char_Widths[font_Last_Char - font_First_Char +1];
 *                  // for each character the separate width in pixels,
 *                  // characters < 128 have an implicit virtual right empty row
 *
 *     uint8_t    font_data[];
 *                  // bit field of all characters
 */


#ifndef _Roosewood26_H
#define _Roosewood26_H

#define _Roosewood26_WIDTH 10
#define _Roosewood26_HEIGHT 26

GLCDFONTDECL(Roosewood26) = {
    0x6E, 0x98, // size
    0x0A, // width
    0x1A, // height
    0x20, // first char
    0x60, // char count
    
    // char widths
    0x07, 0x07, 0x09, 0x0B, 0x0C, 0x12, 0x10, 0x05, 0x06, 0x07, 
    0x08, 0x0A, 0x06, 0x05, 0x05, 0x09, 0x0C, 0x09, 0x0C, 0x0C, 
    0x0C, 0x0C, 0x0B, 0x0E, 0x0D, 0x0D, 0x05, 0x05, 0x0A, 0x0A, 
    0x0A, 0x0B, 0x10, 0x0E, 0x0E, 0x0D, 0x0D, 0x0B, 0x0C, 0x0D, 
    0x0E, 0x08, 0x0B, 0x0E, 0x0C, 0x0F, 0x0D, 0x0D, 0x0D, 0x0F, 
    0x0F, 0x0C, 0x0C, 0x0D, 0x0D, 0x11, 0x0D, 0x0D, 0x0C, 0x07, 
    0x06, 0x08, 0x0A, 0x0C, 0x06, 0x0E, 0x0E, 0x0D, 0x0D, 0x0B, 
    0x0C, 0x0D, 0x0E, 0x08, 0x0B, 0x0E, 0x0C, 0x0F, 0x0D, 0x0D, 
    0x0D, 0x0F, 0x0F, 0x0C, 0x0C, 0x0D, 0x0D, 0x11, 0x0D, 0x0D, 
    0x0C, 0x07, 0x02, 0x07, 0x0B, 0x0C, 
    
    // font data
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0x20 <space>
    0xE0, 0xE0, 0xF0, 0xD0, 0x20, 0xC0, 0x80, 0x03, 0x7F, 0xBD, 0x83, 0xFC, 0xFF, 0x1F, 0x06, 0x09, 0x10, 0x39, 0x3F, 0x3F, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 33
    0xE0, 0xF0, 0x10, 0xE0, 0xE0, 0xF0, 0x10, 0xE0, 0xC0, 0x00, 0x07, 0x0F, 0x0F, 0x07, 0x07, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 34
    0x00, 0x00, 0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0xC0, 0x40, 0x00, 0x60, 0x66, 0xE6, 0x7E, 0x67, 0x66, 0xE6, 0x7E, 0x67, 0x66, 0x06, 0x00, 0x04, 0x07, 0x00, 0x00, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 35
    0x80, 0x40, 0xA0, 0xE0, 0xD8, 0xF8, 0xD8, 0xD0, 0xA0, 0xC0, 0x00, 0x00, 0xC7, 0x2F, 0x3D, 0x3B, 0xCB, 0x1F, 0xAF, 0x2D, 0x6E, 0x9F, 0x3F, 0xDE, 0x03, 0x04, 0x0C, 0x19, 0x3B, 0x60, 0x73, 0x7B, 0x39, 0x38, 0x1C, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 36
    0x80, 0xE0, 0xE0, 0xF0, 0xF0, 0x70, 0xE0, 0xE0, 0xC0, 0x00, 0x80, 0x60, 0x20, 0xE0, 0xE0, 0x40, 0x00, 0x00, 0x07, 0x1F, 0x3F, 0x37, 0x77, 0xF8, 0x7F, 0x1F, 0xCF, 0xE7, 0x39, 0x1E, 0xCF, 0xCB, 0x08, 0x10, 0x30, 0xE0, 0x00, 0x00, 0x00, 0x1C, 0x32, 0x31, 0x3C, 0x0F, 0x03, 0x03, 0x0C, 0x18, 0x17, 0x37, 0x30, 0x38, 0x3C, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 37
    0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xB0, 0xB0, 0x20, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0x7F, 0xBF, 0xC9, 0x1F, 0x6F, 0xD6, 0x3B, 0x67, 0x07, 0x04, 0xC4, 0xE4, 0x78, 0x60, 0x03, 0x05, 0x08, 0x11, 0x13, 0x33, 0x32, 0x38, 0x39, 0x30, 0x10, 0x33, 0x30, 0x38, 0x3F, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 38
    0xE0, 0xF0, 0x10, 0xE0, 0xC0, 0x00, 0x07, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 39
    0x00, 0x80, 0xF0, 0xE8, 0xF8, 0xF8, 0xFF, 0xFC, 0x5F, 0xFF, 0xFF, 0x03, 0x03, 0x0F, 0x34, 0x47, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 40
    0x28, 0xD8, 0xE8, 0xD8, 0x70, 0xE0, 0x80, 0x00, 0x01, 0xFE, 0x5B, 0xFF, 0x19, 0xFF, 0x18, 0x27, 0x60, 0x73, 0x7C, 0x3F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 41
    0x80, 0xE0, 0xA0, 0xD0, 0xF0, 0x70, 0xC0, 0xC0, 0x01, 0x03, 0x07, 0x0B, 0x1F, 0x1F, 0x16, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 42
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x60, 0x60, 0xFE, 0xFE, 0x60, 0x60, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 43
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6E, 0xD1, 0xD1, 0xE2, 0xFE, 0x78, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, // 44
    0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x58, 0x58, 0x58, 0x58, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 45
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x11, 0x31, 0x3E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, // 46
    0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xB0, 0xF0, 0xE0, 0x00, 0xC0, 0x38, 0x16, 0xEF, 0xFD, 0x3F, 0x07, 0x00, 0x1E, 0x31, 0x38, 0x3F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 47
    0x00, 0xC0, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xD0, 0xE0, 0xC0, 0xC0, 0x00, 0xFC, 0x0F, 0xF1, 0x0F, 0xFF, 0xFF, 0x00, 0xFF, 0x0F, 0xFF, 0x8D, 0xFF, 0x00, 0x03, 0x0D, 0x08, 0x11, 0x13, 0x33, 0x38, 0x3A, 0x1D, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 48
    0xC0, 0x40, 0xC0, 0xF0, 0xF0, 0x10, 0xF0, 0xC0, 0x00, 0x03, 0x06, 0xFE, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x00, 0x1C, 0x34, 0x33, 0x30, 0x31, 0x30, 0x37, 0x3F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 49
    0x80, 0xC0, 0x60, 0xA0, 0xB0, 0xB0, 0x70, 0xE0, 0xA0, 0x40, 0x80, 0x00, 0x07, 0xCF, 0xB6, 0x57, 0xBF, 0xE9, 0xDF, 0xEF, 0xEF, 0x78, 0xFF, 0x1E, 0x1F, 0x31, 0x30, 0x3A, 0x19, 0x11, 0x31, 0x31, 0x31, 0x38, 0x3F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 50
    0x80, 0xC0, 0x60, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xA0, 0x40, 0x80, 0x00, 0x83, 0x4F, 0x3E, 0x3C, 0xFF, 0x74, 0xCF, 0x73, 0xEB, 0x3C, 0xDF, 0x9E, 0x03, 0x0C, 0x18, 0x13, 0x13, 0x33, 0x31, 0x31, 0x38, 0x1C, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 51
    0x00, 0x00, 0x00, 0x80, 0xE0, 0x50, 0xF0, 0xF0, 0xF0, 0xC0, 0x00, 0x00, 0xE0, 0x18, 0x2E, 0x33, 0x3D, 0x3F, 0xF9, 0x29, 0x3F, 0x3F, 0x40, 0x80, 0x01, 0x03, 0x03, 0x1F, 0x33, 0x30, 0x31, 0x30, 0x33, 0x3F, 0x3B, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 52
    0x00, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x90, 0xF0, 0xC0, 0xC8, 0x37, 0x1F, 0x15, 0xFF, 0x9F, 0xFB, 0x0D, 0xFF, 0x0D, 0xF9, 0xE1, 0x03, 0x04, 0x08, 0x11, 0x33, 0x33, 0x31, 0x39, 0x18, 0x1E, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 53
    0x00, 0x80, 0xC0, 0x60, 0xE0, 0xD0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0xFC, 0x1F, 0xF1, 0x0F, 0xCE, 0xF7, 0x6F, 0xFD, 0xEF, 0x1F, 0xFF, 0x00, 0x03, 0x0D, 0x18, 0x11, 0x33, 0x32, 0x33, 0x38, 0x1C, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 54
    0xF0, 0x10, 0xF0, 0xE0, 0x60, 0x70, 0xF0, 0x70, 0x60, 0xE0, 0xD0, 0xF0, 0xF0, 0xE0, 0x0F, 0x18, 0x1F, 0x1E, 0x1F, 0xF3, 0x8F, 0x7F, 0x8B, 0xF5, 0xFF, 0x1E, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x30, 0x31, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 55
    0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xA0, 0x40, 0x80, 0x00, 0xC0, 0x27, 0x1F, 0xD8, 0xF7, 0x4F, 0x5A, 0x6D, 0xE7, 0xF3, 0x1C, 0xFF, 0x8F, 0x03, 0x0C, 0x18, 0x11, 0x13, 0x32, 0x32, 0x32, 0x39, 0x39, 0x1E, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 56
    0x80, 0x40, 0xE0, 0x70, 0xB0, 0xF0, 0xF0, 0xE0, 0xA0, 0x40, 0xC0, 0x00, 0x00, 0x87, 0x9F, 0x7F, 0xF0, 0xE7, 0xEF, 0xF7, 0x1F, 0xF1, 0x0E, 0x83, 0xFF, 0xF8, 0x07, 0x0C, 0x18, 0x16, 0x37, 0x36, 0x31, 0x3A, 0x39, 0x1C, 0x1F, 0x0F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 57
    0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0xC4, 0xF8, 0x70, 0x0E, 0x11, 0x31, 0x3E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, // 58
    0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0xC4, 0xF8, 0x60, 0x6E, 0xD1, 0xD1, 0xE2, 0xFE, 0x00, 0x00, 0x40, 0x40, 0x00, // 59
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0xF0, 0xF0, 0x98, 0x98, 0x0C, 0x0C, 0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03, 0x06, 0x06, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 60
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x8C, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 61
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x06, 0x06, 0x0C, 0x0C, 0x98, 0x98, 0xF0, 0xF0, 0x60, 0x0C, 0x06, 0x06, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 62
    0x80, 0xC0, 0x60, 0xD0, 0xF0, 0xF0, 0xF0, 0xE0, 0xA0, 0x40, 0x80, 0x07, 0x0B, 0x1F, 0x7F, 0xE9, 0x9F, 0xEE, 0xF5, 0x7B, 0x1C, 0x1F, 0x00, 0x00, 0x00, 0x0E, 0x11, 0x11, 0x31, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 63
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF8, 0x1C, 0x06, 0xC3, 0xE1, 0x31, 0x10, 0x08, 0xC8, 0xF8, 0x79, 0x01, 0x03, 0x0E, 0xF8, 0x03, 0x0F, 0x1C, 0x30, 0x27, 0x67, 0x42, 0x42, 0x41, 0x47, 0x47, 0x24, 0x14, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 64
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x20, 0xF0, 0xF0, 0xB0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3C, 0x03, 0x8E, 0x9F, 0x9F, 0x3B, 0xED, 0x1F, 0xFF, 0xE0, 0x00, 0x1C, 0x12, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x37, 0x32, 0x31, 0x30, 0x31, 0x33, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 65
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xA0, 0x40, 0x80, 0x00, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xEF, 0xEF, 0x6D, 0xCF, 0x35, 0xE3, 0x18, 0xFF, 0x1E, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x37, 0x36, 0x31, 0x31, 0x38, 0x3C, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 66
    0x00, 0xC0, 0xA0, 0xE0, 0xB0, 0xF0, 0xF0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xC0, 0xFF, 0x07, 0xF8, 0x07, 0xFF, 0xFF, 0x00, 0x01, 0xDE, 0x77, 0x77, 0xFF, 0x3F, 0x00, 0x03, 0x0C, 0x18, 0x11, 0x33, 0x32, 0x33, 0x39, 0x38, 0x1E, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 67
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xC0, 0xC0, 0x00, 0x00, 0xFF, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x81, 0x7F, 0x8F, 0x73, 0x8D, 0xFF, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x33, 0x33, 0x31, 0x38, 0x39, 0x1C, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 68
    0x30, 0xF0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xB0, 0xF0, 0x10, 0xF0, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xFF, 0xFF, 0xFF, 0x35, 0x4C, 0xCF, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x33, 0x33, 0x31, 0x30, 0x30, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 69
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x10, 0xF0, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xEF, 0xEF, 0x78, 0x67, 0x05, 0x0C, 0x0F, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x33, 0x37, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 70
    0x00, 0xC0, 0xA0, 0xE0, 0xB0, 0xF0, 0xF0, 0xE0, 0xE0, 0xF0, 0x10, 0xF0, 0xC0, 0xFF, 0x07, 0xF8, 0x07, 0xFF, 0xFF, 0x70, 0x51, 0x9E, 0x1F, 0x18, 0x9F, 0xFF, 0x00, 0x03, 0x0D, 0x18, 0x11, 0x13, 0x32, 0x32, 0x39, 0x30, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 71
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xB0, 0xB0, 0xF0, 0xF0, 0xB0, 0x90, 0xF0, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xCF, 0xCF, 0xC9, 0xCF, 0xFD, 0x2D, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x34, 0x33, 0x31, 0x30, 0x37, 0x3F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 72
    0x70, 0xB0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xE0, 0x00, 0xFF, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x37, 0x3F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 73
    0x00, 0x00, 0x00, 0x00, 0x70, 0xB0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xC0, 0x20, 0x20, 0x40, 0x80, 0xFF, 0x95, 0x6F, 0x00, 0xFF, 0xFF, 0x03, 0x0C, 0x18, 0x13, 0x33, 0x31, 0x31, 0x38, 0x1C, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 74
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0x70, 0xB0, 0xD0, 0xF0, 0x00, 0xFF, 0x8D, 0x7F, 0x08, 0xD7, 0xAB, 0xC7, 0x1D, 0x7E, 0xEF, 0x83, 0x01, 0x01, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x36, 0x31, 0x30, 0x30, 0x33, 0x36, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 75
    0x70, 0xB0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x0B, 0xFF, 0x00, 0xFF, 0xFF, 0x01, 0xC0, 0x40, 0x40, 0xC0, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x31, 0x33, 0x32, 0x31, 0x30, 0x30, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 76
    0x70, 0xB0, 0xF0, 0x70, 0xF0, 0x90, 0x60, 0xC0, 0xB0, 0x70, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0xFF, 0x0F, 0xFF, 0xCC, 0x17, 0xE8, 0x0F, 0x8F, 0xFC, 0xFB, 0x3B, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x30, 0x33, 0x37, 0x3C, 0x31, 0x30, 0x3F, 0x33, 0x31, 0x30, 0x37, 0x3F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 77
    0x70, 0xB0, 0xF0, 0xF0, 0xF0, 0xB0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xD0, 0xF0, 0x00, 0xFF, 0x0F, 0xFF, 0xF6, 0xC9, 0x36, 0xEB, 0x1F, 0x0F, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x30, 0x37, 0x3F, 0x39, 0x37, 0x09, 0x30, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 78
    0x00, 0xC0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xA0, 0xC0, 0x80, 0x00, 0xFF, 0x0B, 0xF8, 0x2F, 0xFF, 0xFF, 0x00, 0xFF, 0x0F, 0xF3, 0x0C, 0xFF, 0xFE, 0x01, 0x06, 0x09, 0x18, 0x13, 0x37, 0x34, 0x33, 0x39, 0x38, 0x1E, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 79
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xA0, 0x40, 0x80, 0x00, 0xFF, 0x8D, 0x7F, 0x08, 0xCF, 0xCF, 0xCD, 0xCF, 0xEF, 0xE3, 0x70, 0x7F, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x38, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 80
    0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0xFF, 0x0B, 0xF8, 0x6F, 0x3F, 0x3F, 0x20, 0x7F, 0x0F, 0xF3, 0x0C, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0x0D, 0x18, 0x17, 0x17, 0x35, 0x32, 0x31, 0x20, 0x20, 0x63, 0x71, 0x7F, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 81
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xE0, 0x40, 0x80, 0x00, 0x00, 0x00, 0xFF, 0x8D, 0x7F, 0x08, 0xCF, 0xCF, 0x4D, 0xB7, 0xE9, 0x37, 0xF8, 0xFF, 0x5E, 0x80, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x33, 0x37, 0x38, 0x07, 0x19, 0x10, 0x31, 0x30, 0x38, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 82
    0x80, 0x40, 0xE0, 0xF0, 0xF0, 0x70, 0x70, 0x70, 0xD0, 0xF0, 0xF0, 0xC0, 0xCF, 0x57, 0xAF, 0x6D, 0x4B, 0xCA, 0x9C, 0xAD, 0xC6, 0x0B, 0x1F, 0xF7, 0x1F, 0x30, 0x38, 0x39, 0x12, 0x12, 0x32, 0x31, 0x38, 0x3C, 0x1E, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 83
    0xF0, 0xF0, 0xF0, 0xF0, 0xB0, 0xF0, 0xF0, 0x70, 0xB0, 0x70, 0xF0, 0xF0, 0x0F, 0x1B, 0x1F, 0x1F, 0xFF, 0x9B, 0x6F, 0x00, 0xFF, 0xF8, 0x0B, 0x1F, 0x00, 0x00, 0x1C, 0x34, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 84
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xF0, 0xB0, 0xF0, 0xB0, 0x90, 0xF0, 0x00, 0xFF, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x01, 0xFF, 0x0F, 0xFF, 0xFF, 0x01, 0x00, 0x01, 0x06, 0x09, 0x10, 0x13, 0x37, 0x34, 0x3B, 0x3C, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 85
    0x70, 0x70, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xB0, 0xF0, 0x30, 0xD0, 0xF0, 0x00, 0x00, 0x1F, 0x33, 0xEC, 0x03, 0xFF, 0x19, 0x8F, 0xF8, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x07, 0x09, 0x10, 0x30, 0x38, 0x3F, 0x3F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 86
    0x70, 0x70, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xF0, 0xF0, 0xF0, 0x60, 0xF0, 0xB0, 0xF0, 0xB0, 0xD0, 0xF0, 0x00, 0x00, 0x07, 0x3B, 0xEC, 0x03, 0x3F, 0x0C, 0xFF, 0x3B, 0xEC, 0x3F, 0x0F, 0xC0, 0xFF, 0x3F, 0x01, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x10, 0x30, 0x3F, 0x3F, 0x1F, 0x11, 0x30, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 87
    0x30, 0x30, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x01, 0x85, 0x76, 0x29, 0x92, 0x6F, 0x94, 0x3F, 0xDE, 0x87, 0x03, 0x01, 0x1C, 0x36, 0x31, 0x30, 0x36, 0x3F, 0x36, 0x30, 0x31, 0x30, 0x33, 0x36, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 88
    0x30, 0x30, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x01, 0x0E, 0xFB, 0xB4, 0x6B, 0xF4, 0xFB, 0xFE, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 89
    0xF0, 0xF0, 0xF0, 0xB0, 0xB0, 0xB0, 0x30, 0xF0, 0xF0, 0x70, 0x90, 0xF0, 0x07, 0x0D, 0xEF, 0x9F, 0x6F, 0xD7, 0xED, 0xFB, 0xFD, 0x5F, 0x8F, 0x03, 0x1E, 0x13, 0x31, 0x30, 0x33, 0x33, 0x33, 0x32, 0x31, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 90
    0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0x68, 0x78, 0xFF, 0xFB, 0x2B, 0xFF, 0xFF, 0x00, 0x00, 0x3F, 0x61, 0x60, 0x6F, 0x6F, 0x68, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 91
    0x30, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1F, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 92
    0x38, 0x78, 0xF8, 0xF8, 0xF8, 0x18, 0xF8, 0xF0, 0x00, 0x00, 0xFF, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x38, 0x68, 0x6F, 0x60, 0x61, 0x60, 0x7F, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 93
    0x00, 0x00, 0x00, 0xC0, 0xE0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x20, 0x38, 0x1E, 0x03, 0x00, 0x00, 0x07, 0x1E, 0x38, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 94
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 95
    0x06, 0x0B, 0x0E, 0x0E, 0x0C, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 96
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x20, 0xF0, 0xF0, 0xB0, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x3C, 0x03, 0x8E, 0x9F, 0x9F, 0x3B, 0xED, 0x1F, 0xFF, 0xE0, 0x00, 0x1C, 0x12, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x37, 0x32, 0x31, 0x30, 0x31, 0x33, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 97
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xA0, 0x40, 0x80, 0x00, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xEF, 0xEF, 0x6D, 0xCF, 0x35, 0xE3, 0x18, 0xFF, 0x1E, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x37, 0x36, 0x31, 0x31, 0x38, 0x3C, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 98
    0x00, 0xC0, 0xA0, 0xE0, 0xB0, 0xF0, 0xF0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xC0, 0xFF, 0x07, 0xF8, 0x07, 0xFF, 0xFF, 0x00, 0x01, 0xDE, 0x77, 0x77, 0xFF, 0x3F, 0x00, 0x03, 0x0C, 0x18, 0x11, 0x33, 0x32, 0x33, 0x39, 0x38, 0x1E, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 99
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xC0, 0xC0, 0x00, 0x00, 0xFF, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x81, 0x7F, 0x8F, 0x73, 0x8D, 0xFF, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x33, 0x33, 0x31, 0x38, 0x39, 0x1C, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 100
    0x30, 0xF0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xB0, 0xF0, 0x10, 0xF0, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xFF, 0xFF, 0xFF, 0x35, 0x4C, 0xCF, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x33, 0x33, 0x31, 0x30, 0x30, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 101
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x10, 0xF0, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xEF, 0xEF, 0x78, 0x67, 0x05, 0x0C, 0x0F, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x33, 0x37, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 102
    0x00, 0xC0, 0xA0, 0xE0, 0xB0, 0xF0, 0xF0, 0xE0, 0xE0, 0xF0, 0x10, 0xF0, 0xC0, 0xFF, 0x07, 0xF8, 0x07, 0xFF, 0xFF, 0x70, 0x51, 0x9E, 0x1F, 0x18, 0x9F, 0xFF, 0x00, 0x03, 0x0D, 0x18, 0x11, 0x13, 0x32, 0x32, 0x39, 0x30, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 103
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xB0, 0xB0, 0xF0, 0xF0, 0xB0, 0x90, 0xF0, 0x00, 0xFF, 0x0D, 0xFF, 0x08, 0xCF, 0xCF, 0xC9, 0xCF, 0xFD, 0x2D, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x34, 0x33, 0x31, 0x30, 0x37, 0x3F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 104
    0x70, 0xB0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xE0, 0x00, 0xFF, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x37, 0x3F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 105
    0x00, 0x00, 0x00, 0x00, 0x70, 0xB0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xC0, 0x20, 0x20, 0x40, 0x80, 0xFF, 0x95, 0x6F, 0x00, 0xFF, 0xFF, 0x03, 0x0C, 0x18, 0x13, 0x33, 0x31, 0x31, 0x38, 0x1C, 0x1F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 106
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0x70, 0xB0, 0xD0, 0xF0, 0x00, 0xFF, 0x8D, 0x7F, 0x08, 0xD7, 0xAB, 0xC7, 0x1D, 0x7E, 0xEF, 0x83, 0x01, 0x01, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x36, 0x31, 0x30, 0x30, 0x33, 0x36, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 107
    0x70, 0xB0, 0xF0, 0xF0, 0x70, 0xF0, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x0B, 0xFF, 0x00, 0xFF, 0xFF, 0x01, 0xC0, 0x40, 0x40, 0xC0, 0x1C, 0x33, 0x30, 0x31, 0x30, 0x31, 0x33, 0x32, 0x31, 0x30, 0x30, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 108
    0x70, 0xB0, 0xF0, 0x70, 0xF0, 0x90, 0x60, 0xC0, 0xB0, 0x70, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0xFF, 0x0F, 0xFF, 0xCC, 0x17, 0xE8, 0x0F, 0x8F, 0xFC, 0xFB, 0x3B, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x30, 0x33, 0x37, 0x3C, 0x31, 0x30, 0x3F, 0x33, 0x31, 0x30, 0x37, 0x3F, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 109
    0x70, 0xB0, 0xF0, 0xF0, 0xF0, 0xB0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xD0, 0xF0, 0x00, 0xFF, 0x0F, 0xFF, 0xF6, 0xC9, 0x36, 0xEB, 0x1F, 0x0F, 0xFF, 0xFF, 0x01, 0x1C, 0x33, 0x30, 0x37, 0x3F, 0x39, 0x37, 0x09, 0x30, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 110
    0x00, 0xC0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xA0, 0xC0, 0x80, 0x00, 0xFF, 0x0B, 0xF8, 0x2F, 0xFF, 0xFF, 0x00, 0xFF, 0x0F, 0xF3, 0x0C, 0xFF, 0xFE, 0x01, 0x06, 0x09, 0x18, 0x13, 0x37, 0x34, 0x33, 0x39, 0x38, 0x1E, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 111
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xA0, 0x40, 0x80, 0x00, 0xFF, 0x8D, 0x7F, 0x08, 0xCF, 0xCF, 0xCD, 0xCF, 0xEF, 0xE3, 0x70, 0x7F, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x38, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 112
    0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0xFF, 0x0B, 0xF8, 0x6F, 0x3F, 0x3F, 0x20, 0x7F, 0x0F, 0xF3, 0x0C, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x03, 0x0D, 0x18, 0x17, 0x17, 0x35, 0x32, 0x31, 0x20, 0x20, 0x63, 0x71, 0x7F, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 113
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xB0, 0xB0, 0x70, 0xE0, 0xE0, 0x40, 0x80, 0x00, 0x00, 0x00, 0xFF, 0x8D, 0x7F, 0x08, 0xCF, 0xCF, 0x4D, 0xB7, 0xE9, 0x37, 0xF8, 0xFF, 0x5E, 0x80, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x33, 0x37, 0x38, 0x07, 0x19, 0x10, 0x31, 0x30, 0x38, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 114
    0x80, 0x40, 0xE0, 0xF0, 0xF0, 0x70, 0x70, 0x70, 0xD0, 0xF0, 0xF0, 0xC0, 0xCF, 0x57, 0xAF, 0x6D, 0x4B, 0xCA, 0x9C, 0xAD, 0xC6, 0x0B, 0x1F, 0xF7, 0x1F, 0x30, 0x38, 0x39, 0x12, 0x12, 0x32, 0x31, 0x38, 0x3C, 0x1E, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 115
    0xF0, 0xF0, 0xF0, 0xF0, 0xB0, 0xF0, 0xF0, 0x70, 0xB0, 0x70, 0xF0, 0xF0, 0x0F, 0x1B, 0x1F, 0x1F, 0xFF, 0x9B, 0x6F, 0x00, 0xFF, 0xF8, 0x0B, 0x1F, 0x00, 0x00, 0x1C, 0x34, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 116
    0x70, 0xB0, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xF0, 0xB0, 0xF0, 0xB0, 0x90, 0xF0, 0x00, 0xFF, 0x1B, 0xEF, 0x00, 0xFF, 0xFF, 0x01, 0xFF, 0x0F, 0xFF, 0xFF, 0x01, 0x00, 0x01, 0x06, 0x09, 0x10, 0x13, 0x37, 0x34, 0x3B, 0x3C, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 117
    0x70, 0x70, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xB0, 0xF0, 0x30, 0xD0, 0xF0, 0x00, 0x00, 0x1F, 0x33, 0xEC, 0x03, 0xFF, 0x19, 0x8F, 0xF8, 0xFF, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x07, 0x09, 0x10, 0x30, 0x38, 0x3F, 0x3F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 118
    0x70, 0x70, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xF0, 0xF0, 0xF0, 0x60, 0xF0, 0xB0, 0xF0, 0xB0, 0xD0, 0xF0, 0x00, 0x00, 0x07, 0x3B, 0xEC, 0x03, 0x3F, 0x0C, 0xFF, 0x3B, 0xEC, 0x3F, 0x0F, 0xC0, 0xFF, 0x3F, 0x01, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x10, 0x30, 0x3F, 0x3F, 0x1F, 0x11, 0x30, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 119
    0x30, 0x30, 0xF0, 0xF0, 0x30, 0xB0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x01, 0x85, 0x76, 0x29, 0x92, 0x6F, 0x94, 0x3F, 0xDE, 0x87, 0x03, 0x01, 0x1C, 0x36, 0x31, 0x30, 0x36, 0x3F, 0x36, 0x30, 0x31, 0x30, 0x33, 0x36, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 120
    0x30, 0x30, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0x30, 0xF0, 0xF0, 0xF0, 0xE0, 0x00, 0x00, 0x01, 0x0E, 0xFB, 0xB4, 0x6B, 0xF4, 0xFB, 0xFE, 0x0F, 0x03, 0x01, 0x00, 0x00, 0x00, 0x1C, 0x33, 0x31, 0x30, 0x30, 0x37, 0x3F, 0x30, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 121
    0xF0, 0xF0, 0xF0, 0xB0, 0xB0, 0xB0, 0x30, 0xF0, 0xF0, 0x70, 0x90, 0xF0, 0x07, 0x0D, 0xEF, 0x9F, 0x6F, 0xD7, 0xED, 0xFB, 0xFD, 0x5F, 0x8F, 0x03, 0x1E, 0x13, 0x31, 0x30, 0x33, 0x33, 0x33, 0x32, 0x31, 0x30, 0x3F, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 122
    0x00, 0xC0, 0xB0, 0xF0, 0xB8, 0xF8, 0xF8, 0x78, 0xEF, 0xF9, 0x17, 0xFF, 0x1F, 0x00, 0x00, 0x0F, 0x13, 0x60, 0x47, 0x5F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 123
    0xFC, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xC0, // 124
    0x38, 0x78, 0xF0, 0xE0, 0x20, 0xC0, 0x00, 0x00, 0x00, 0xFF, 0x87, 0x78, 0xDF, 0xFF, 0x70, 0xD8, 0xC7, 0xE3, 0xF8, 0x7F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 125
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x60, 0x30, 0x30, 0x30, 0x60, 0xC0, 0xC0, 0xC0, 0x70, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 126
    0xF8, 0x38, 0x68, 0x88, 0x08, 0x08, 0x08, 0x08, 0x88, 0x68, 0x38, 0xF8, 0xFF, 0x00, 0x00, 0xC1, 0x63, 0x1C, 0x1C, 0x63, 0xC1, 0x00, 0x00, 0xFF, 0x0F, 0x0E, 0x0B, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0B, 0x0E, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // 127
    
};

#endif

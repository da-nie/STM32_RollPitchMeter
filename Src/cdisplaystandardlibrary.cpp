//****************************************************************************************************
//������������ ����������
//****************************************************************************************************
#include "cdisplaystandardlibrary.h"

//****************************************************************************************************
//���������� ����������
//****************************************************************************************************

//****************************************************************************************************
//���������
//****************************************************************************************************

static const unsigned char Font8x14[224][14]=
{
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},//' '=0x20 
 { 0x00, 0x10, 0x38, 0x38, 0x38, 0x10, 0x10, 0x10, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00},//'!'=0x21 
 { 0x00, 0x66, 0x66, 0x22, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},//'"'=0x22 
 { 0x00, 0x44, 0x44, 0xFE, 0xFE, 0x44, 0x44, 0xFE, 0xFE, 0x44, 0x44, 0x00, 0x00, 0x00},//'#'=0x23 
 { 0x18, 0x18, 0x7C, 0xC6, 0xC2, 0xC0, 0x7C, 0x06, 0x06, 0x86, 0xC6, 0x7C, 0x18, 0x18}, 
 { 0x00, 0x00, 0x00, 0xC2, 0xC6, 0x0C, 0x18, 0x30, 0x60, 0xC6, 0x86, 0x00, 0x00, 0x00}, 
 { 0x00, 0x38, 0x6C, 0x6C, 0x38, 0x76, 0xDC, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00}, 
 { 0x00, 0x30, 0x30, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x7C, 0x7C, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x30, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00}, 
 { 0x02, 0x06, 0x04, 0x0C, 0x08, 0x18, 0x10, 0x30, 0x20, 0x60, 0x40, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0xCE, 0xDE, 0xF6, 0xE6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x18, 0x38, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0xC6, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0x06, 0x06, 0x3C, 0x06, 0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x0C, 0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0xC0, 0xC0, 0xC0, 0xFC, 0x06, 0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x38, 0x60, 0xC0, 0xC0, 0xFC, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0xC6, 0x06, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x06, 0x06, 0x0C, 0x78, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0x0C, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xDE, 0xDE, 0xDE, 0xDC, 0xC0, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x10, 0x38, 0x6C, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00}, 
 { 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xC0, 0xC0, 0xC2, 0x66, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xF8, 0x6C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x6C, 0xF8, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00}, 
 { 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xDE, 0xC6, 0xC6, 0x66, 0x3A, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0xCC, 0xCC, 0xCC, 0x78, 0x00, 0x00, 0x00}, 
 { 0x00, 0xE6, 0x66, 0x66, 0x6C, 0x78, 0x78, 0x6C, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xF0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xEE, 0xFE, 0xFE, 0xD6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xE6, 0xF6, 0xFE, 0xDE, 0xCE, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xD6, 0xDE, 0x7C, 0x0C, 0x0E, 0x00}, 
 { 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0x60, 0x38, 0x0C, 0x06, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7E, 0x7E, 0x5A, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x6C, 0x38, 0x10, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xD6, 0xD6, 0xD6, 0xFE, 0xEE, 0x6C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0x6C, 0x7C, 0x38, 0x38, 0x7C, 0x6C, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0xC6, 0x86, 0x0C, 0x18, 0x30, 0x60, 0xC2, 0xC6, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x00, 0x3C, 0x3C, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x40, 0x60, 0x20, 0x30, 0x10, 0x18, 0x08, 0x0C, 0x04, 0x06, 0x02, 0x00, 0x00, 0x00}, 
 { 0x00, 0x3C, 0x3C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x3C, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x10, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00}, 
 { 0x00, 0x20, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x78, 0x0C, 0x7C, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00}, 
 { 0x00, 0xE0, 0x60, 0x60, 0x78, 0x6C, 0x66, 0x66, 0x66, 0x66, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC0, 0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x1C, 0x0C, 0x0C, 0x3C, 0x6C, 0xCC, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xFE, 0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x38, 0x6C, 0x64, 0x60, 0xF0, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x76, 0xCC, 0xCC, 0xCC, 0xCC, 0x7C, 0x0C, 0xCC, 0x78, 0x00}, 
 { 0x00, 0xE0, 0x60, 0x60, 0x6C, 0x76, 0x66, 0x66, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x18, 0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x06, 0x06, 0x00, 0x0E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x66, 0x66, 0x3C, 0x00}, 
 { 0x00, 0xE0, 0x60, 0x60, 0x66, 0x6C, 0x78, 0x78, 0x6C, 0x66, 0xE6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xEC, 0xFE, 0xD6, 0xD6, 0xD6, 0xD6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xDC, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xDC, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7C, 0x60, 0xF0, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x76, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x7C, 0x0C, 0x1E, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xDC, 0x76, 0x66, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0x60, 0x38, 0x0C, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x10, 0x30, 0x30, 0xFC, 0x30, 0x30, 0x30, 0x30, 0x36, 0x1C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xD6, 0xD6, 0xD6, 0xFE, 0x6C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0x6C, 0x38, 0x38, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0xC6, 0x7C, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xFE, 0xCC, 0x18, 0x30, 0x60, 0xC6, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x00, 0x0E, 0x18, 0x18, 0x18, 0x70, 0x18, 0x18, 0x18, 0x18, 0x0E, 0x00, 0x00, 0x00}, 
 { 0x00, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00}, 
 { 0x00, 0x70, 0x18, 0x18, 0x18, 0x0E, 0x18, 0x18, 0x18, 0x18, 0x70, 0x00, 0x00, 0x00}, 
 { 0x00, 0x76, 0xDC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x10, 0x38, 0x6C, 0xC6, 0xC6, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03}, 
 { 0x00, 0x00, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 
 { 0x00, 0x18, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 
 { 0x00, 0x00, 0xC0, 0xE1, 0xF1, 0xF9, 0xF9, 0xF9, 0xFD, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE}, 
 { 0x00, 0x00, 0x00, 0xC0, 0x20, 0xA0, 0xE0, 0xA0, 0xE0, 0x20, 0xC0, 0x00, 0x00, 0x00}, 
 { 0x07, 0x0F, 0x0F, 0x0F, 0x1F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x7F, 0x7F}, 
 { 0xFB, 0xF9, 0xF1, 0xF0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x80, 0xC0}, 
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x7E, 0x3E, 0x3E, 0x3C, 0x1C, 0x1E}, 
 { 0xFF, 0xDF, 0xCF, 0x8F, 0x87, 0x87, 0x07, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00}, 
 { 0x00, 0x80, 0x80, 0x80, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0xF8}, 
 { 0x7F, 0x7F, 0x7C, 0x7C, 0x7C, 0x78, 0x78, 0x78, 0x70, 0x70, 0x70, 0x60, 0x60, 0x60}, 
 { 0xE0, 0xE0, 0xE0, 0xF0, 0xF0, 0xF0, 0x78, 0x78, 0x78, 0x3C, 0x3C, 0x3C, 0x1E, 0x1E}, 
 { 0x1F, 0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x07, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00}, 
 { 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0xC0, 0xC0, 0xC0, 0xE0, 0xE0, 0xE0, 0xF0, 0xF0}, 
 { 0x78, 0x78, 0x3C, 0x3C, 0x3C, 0x1E, 0x1E, 0x1E, 0x1F, 0x0F, 0x0F, 0x0F, 0x07, 0x07}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x7F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 
 { 0x00, 0x00, 0x80, 0x80, 0x80, 0xC0, 0xC1, 0xC1, 0xE1, 0xE3, 0xE3, 0xF3, 0xF7, 0xFF}, 
 { 0x78, 0x78, 0x78, 0x7C, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 
 { 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80}, 
 { 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00}, 
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x18, 0x00, 0x00}, 
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xC0, 0x00, 0x00, 0x00}, 
 { 0x80, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x18, 0x3C, 0x24, 0x24, 0x24, 0x24, 0x3C, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x10, 0x10, 0x7C, 0x7C, 0x10, 0x10, 0x00, 0x7C, 0x7C, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x28, 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x3C, 0x66, 0x42, 0x81, 0x99, 0xBD, 0xA1, 0xA1, 0xBD, 0x99, 0x81, 0x42, 0x66, 0x3C}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x66, 0xCC, 0x66, 0x33, 0x00, 0x00, 0x00, 0x00}, 
 { 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA}, 
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 
 { 0x3C, 0x66, 0x42, 0x81, 0xB9, 0xBD, 0xA5, 0xB9, 0xBD, 0xA5, 0x81, 0x42, 0x66, 0x3C}, 
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}, 
 { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}, 
 { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x1F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}, 
 { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xF0, 0xF0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}, 
 { 0x00, 0x28, 0x28, 0x00, 0x7C, 0xC6, 0xFE, 0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}, 
 { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0xCC, 0x66, 0x33, 0x66, 0xCC, 0x00, 0x00, 0x00, 0x00}, 
 { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xFF, 0xFF, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10}, 
 { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, 
 { 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0}, 
 { 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F}, 
 { 0x00, 0x1E, 0x36, 0x66, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0x62, 0x62, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0x62, 0x62, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00}, 
 { 0x00, 0x1E, 0x36, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0xFF, 0xC3, 0x81, 0x00}, 
 { 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x00, 0xD6, 0xD6, 0x54, 0x54, 0x7C, 0x7C, 0x54, 0xD6, 0xD6, 0xD6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0x06, 0x06, 0x3C, 0x06, 0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xCE, 0xCE, 0xD6, 0xE6, 0xE6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x28, 0x10, 0xC6, 0xC6, 0xCE, 0xCE, 0xD6, 0xE6, 0xE6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xE6, 0x66, 0x6C, 0x6C, 0x78, 0x78, 0x6C, 0x6C, 0x66, 0xE6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x1E, 0x36, 0x66, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xEE, 0xFE, 0xFE, 0xD6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00}, 
 { 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xC0, 0xC0, 0xC2, 0x66, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7E, 0x5A, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x3C, 0x18, 0x7E, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0x7E, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0x6C, 0x7C, 0x38, 0x38, 0x7C, 0x6C, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xFE, 0x06, 0x06, 0x00}, 
 { 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00}, 
 { 0x00, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xFF, 0x00, 0x00, 0x00}, 
 { 0x00, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xFF, 0x03, 0x03, 0x00}, 
 { 0x00, 0xF8, 0xB0, 0x30, 0x30, 0x3C, 0x36, 0x36, 0x36, 0x36, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xC3, 0xC3, 0xC3, 0xC3, 0xF3, 0xDB, 0xDB, 0xDB, 0xDB, 0xF3, 0x00, 0x00, 0x00}, 
 { 0x00, 0xF0, 0x60, 0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00}, 
 { 0x00, 0x7C, 0xC6, 0x06, 0x26, 0x3E, 0x26, 0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0xCE, 0xDB, 0xDB, 0xDB, 0xFB, 0xDB, 0xDB, 0xDB, 0xDB, 0xCE, 0x00, 0x00, 0x00}, 
 { 0x00, 0x3F, 0x66, 0x66, 0x66, 0x3E, 0x3E, 0x66, 0x66, 0x66, 0xE7, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x78, 0x0C, 0x7C, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00}, 
 { 0x02, 0x06, 0x3C, 0x60, 0x60, 0x7C, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xFC, 0x66, 0x66, 0x7C, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7E, 0x32, 0x32, 0x30, 0x30, 0x30, 0x78, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x1E, 0x36, 0x36, 0x66, 0x66, 0x66, 0xFF, 0xC3, 0xC3, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xFE, 0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xD6, 0xD6, 0x54, 0x7C, 0x54, 0xD6, 0xD6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x3C, 0x66, 0x06, 0x0C, 0x06, 0x66, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xCE, 0xD6, 0xE6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x28, 0x10, 0xC6, 0xC6, 0xCE, 0xD6, 0xE6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xE6, 0x6C, 0x78, 0x78, 0x6C, 0x66, 0xE6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x1E, 0x36, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xEE, 0xFE, 0xFE, 0xD6, 0xD6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xDC, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7C, 0x60, 0xF0, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC0, 0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7E, 0x5A, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0xC6, 0x7C, 0x00}, 
 { 0x00, 0x00, 0x00, 0x3C, 0x18, 0x7E, 0xDB, 0xDB, 0xDB, 0xDB, 0x7E, 0x18, 0x3C, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0x6C, 0x38, 0x38, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xFE, 0x06, 0x06, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x06, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xD6, 0xD6, 0xD6, 0xD6, 0xD6, 0xD6, 0xFE, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xD6, 0xD6, 0xD6, 0xD6, 0xD6, 0xD6, 0xFE, 0x03, 0x03, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xF8, 0xB0, 0x30, 0x3E, 0x33, 0x33, 0x7E, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xF6, 0xDE, 0xDE, 0xF6, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xF0, 0x60, 0x60, 0x7C, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x3C, 0x66, 0x06, 0x1E, 0x06, 0x66, 0x3C, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0xCE, 0xDB, 0xDB, 0xFB, 0xDB, 0xDB, 0xCE, 0x00, 0x00, 0x00}, 
 { 0x00, 0x00, 0x00, 0x00, 0x7E, 0xCC, 0xCC, 0xFC, 0x6C, 0xCC, 0xCE, 0x00, 0x00, 0x00}
};

//****************************************************************************************************
//����������������
//****************************************************************************************************

//****************************************************************************************************
//����������� � ����������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//�����������
//----------------------------------------------------------------------------------------------------
CDisplayStandardLibrary::CDisplayStandardLibrary(IDisplay *iDisplay_Ptr_Set,bool orientation_vertical)
{ 
 iDisplay_Ptr=iDisplay_Ptr_Set;
 PrintYPosition=0;
 ClearColor=iDisplay_Ptr->COLOR_BLACK;
 OrientationVertical=orientation_vertical;
}
//----------------------------------------------------------------------------------------------------
//����������
//----------------------------------------------------------------------------------------------------
CDisplayStandardLibrary::~CDisplayStandardLibrary()
{
}

//****************************************************************************************************
//�������� �������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//����� ������� � ������� � ������������ ����������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::PutSymbolVertical(int32_t x,int32_t y,uint8_t symbol,uint16_t color)
{
 for(int32_t ys=0;ys<FONT_HEIGHT;ys++)
 {
  uint8_t byte=Font8x14[symbol][ys];
  uint8_t mask=128;
  for(int32_t xs=0;xs<FONT_WIDTH;xs++,mask>>=1)
  {   
   if (x+xs<0 || x+xs>=iDisplay_Ptr->DISPLAY_WIDTH) continue;
   if (y+ys<0 || y+ys>=iDisplay_Ptr->DISPLAY_HEIGHT) continue;
   if (byte&mask) iDisplay_Ptr->PutPixel(x+xs,y+ys,color);
  }
 }
}

//----------------------------------------------------------------------------------------------------
//����� ������� � ������� � �������������� ����������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::PutSymbolHorizontal(int32_t x,int32_t y,uint8_t symbol,uint16_t color)
{
 for(int32_t ys=0;ys<FONT_HEIGHT;ys++)
 {
  uint8_t byte=Font8x14[symbol][ys];
  uint8_t mask=128;
  for(int32_t xs=0;xs<FONT_WIDTH;xs++,mask>>=1)
  {
   if (x+xs<0 || x+xs>=iDisplay_Ptr->DISPLAY_HEIGHT) continue;
   if (y+ys<0 || y+ys>=iDisplay_Ptr->DISPLAY_WIDTH) continue;
   if (byte&mask) iDisplay_Ptr->PutPixel(iDisplay_Ptr->DISPLAY_WIDTH-(y+ys),x+xs,color);
  }
 }
}

//----------------------------------------------------------------------------------------------------
//��������� ����� ������������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::DrawTriangleLine(int32_t y,int32_t x1,int32_t x2,uint16_t color)
{
 if (x2>=iDisplay_Ptr->DISPLAY_HEIGHT) x2=iDisplay_Ptr->DISPLAY_HEIGHT-1;
 if (x1<0)
 {
  x1=0;
 }
 for(float x=x1;x<=x2;x++)
 {
  iDisplay_Ptr->PutPixel(iDisplay_Ptr->DISPLAY_WIDTH-1-y,x,color);
 }
}

//****************************************************************************************************
//�������� �������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//����� ������� � �������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::PutSymbol(int32_t x,int32_t y,char symbol,uint16_t color)
{
 uint8_t s=(uint8_t)(symbol);
 if (s<=32) return;
 s-=32;
 if (s>223) return;
 //������ ������
 if (OrientationVertical==true) PutSymbolVertical(x,y,s,color);	
                           else PutSymbolHorizontal(x,y,s,color);	
}
//----------------------------------------------------------------------------------------------------
//����� ������� � �������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::PutString(int32_t x,int32_t y,const char *string,uint16_t color)
{
 uint32_t s=0;	
 while((*string)!=0)
 {
  PutSymbol(x,y,*string,color);
	x+=FONT_WIDTH;
	string++;
	s++;
 }
}
//----------------------------------------------------------------------------------------------------
//������� ����� � ������� �������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::Print(const char *string,uint16_t color)
{
 if (OrientationVertical==true)
 {
	if (PrintYPosition+FONT_HEIGHT>=iDisplay_Ptr->DISPLAY_HEIGHT) Clear(ClearColor);
 }
 else
 {
	if (PrintYPosition+FONT_HEIGHT>=iDisplay_Ptr->DISPLAY_WIDTH) Clear(ClearColor);
 }
 PutString(0,PrintYPosition,string,color);
 PrintYPosition+=FONT_HEIGHT;
}
//----------------------------------------------------------------------------------------------------
//�������� ������� ������� � �������� ������� 
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::Clear(uint16_t color)
{
 PrintYPosition=0;
 ClearColor=color;	
 iDisplay_Ptr->SetWindow(0,0,iDisplay_Ptr->DISPLAY_WIDTH-1,iDisplay_Ptr->DISPLAY_HEIGHT-1);
 for(uint32_t y=0;y<iDisplay_Ptr->DISPLAY_HEIGHT;y++)
 {
  for(uint32_t x=0;x<iDisplay_Ptr->DISPLAY_WIDTH;x++) iDisplay_Ptr->OutColor(color);
 }
}
//----------------------------------------------------------------------------------------------------
//���������� �����
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::DrawLine(int32_t x1,int32_t y1,int32_t x2,int32_t y2,uint16_t color)
{
 if (x1<0 && x2<0) return;
 if (y1<0 && y2<0) return;
 if (y1>=iDisplay_Ptr->DISPLAY_WIDTH && y2>=iDisplay_Ptr->DISPLAY_WIDTH) return;
 if (x1>=iDisplay_Ptr->DISPLAY_HEIGHT && x2>=iDisplay_Ptr->DISPLAY_HEIGHT) return;
 int32_t dx=x2-x1;
 int32_t dy=y2-y1; 
 if (dx<0) dx=-dx;
 if (dy<0) dy=-dy;
 int32_t sx=x2>=x1?1:-1;
 int32_t sy=y2>=y1?1:-1;
 if (dy<=dx)
 {
  int32_t d=(dy<<1)-dx;
  int32_t d1=dy<<1;
  int32_t d2=(dy-dx)<<1;
	 
  if (x1>=0 && x1<iDisplay_Ptr->DISPLAY_HEIGHT && y1>=0 && y1<iDisplay_Ptr->DISPLAY_WIDTH) iDisplay_Ptr->PutPixel(iDisplay_Ptr->DISPLAY_WIDTH-1-y1,x1,color);
	
  for(int32_t x=x1+sx,y=y1,i=1;i<=dx;i++,x+=sx)
  {
   if (d>0)
   {
    d+=d2;
    y+=sy;
   }
   else d+=d1;
	 if (x>=0 && x<iDisplay_Ptr->DISPLAY_HEIGHT && y>=0 && y<iDisplay_Ptr->DISPLAY_WIDTH) iDisplay_Ptr->PutPixel(iDisplay_Ptr->DISPLAY_WIDTH-1-y,x,color);
  }
 }
 else
 {
  int32_t d=(dx<<1)-dy;
  int32_t d1=dx<<1;
  int32_t d2=(dx-dy)<<1;
  if (x1>=0 && x1<iDisplay_Ptr->DISPLAY_HEIGHT && y1>=0 && y1<iDisplay_Ptr->DISPLAY_WIDTH) iDisplay_Ptr->PutPixel(iDisplay_Ptr->DISPLAY_WIDTH-1-y1,x1,color);
  for(int32_t x=x1,y=y1+sy,i=1;i<=dy;i++,y+=sy)
  {
   if (d>0)
   {
    d+=d2;
    x+=sx;
   }
   else d+=d1;
	 if (x>=0 && x<iDisplay_Ptr->DISPLAY_HEIGHT && y>=0 && y<iDisplay_Ptr->DISPLAY_WIDTH) iDisplay_Ptr->PutPixel(iDisplay_Ptr->DISPLAY_WIDTH-1-y,x,color);
  }
 }
}
//----------------------------------------------------------------------------------------------------
//���������� ����������� �������������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::FillRectangle(int32_t x1,int32_t y1,int32_t x2,int32_t y2,uint16_t color)
{
 if (x1>x2)
 {
  int32_t tmp=x2;
  x2=x1;
  x1=tmp; 	
 }	
 if (y1>y2)
 {
  int32_t tmp=y2;
  y2=y1;
  y1=tmp; 	
 }
	
 if (x1<0 && x2<0) return;
 if (y1<0 && y2<0) return;
 
 if (y1>=iDisplay_Ptr->DISPLAY_WIDTH && y2>=iDisplay_Ptr->DISPLAY_WIDTH) return;
 if (x1>=iDisplay_Ptr->DISPLAY_HEIGHT && x2>=iDisplay_Ptr->DISPLAY_HEIGHT) return;
 
 if (y1<0) y1=0;
 if (y2>=iDisplay_Ptr->DISPLAY_WIDTH) y2=iDisplay_Ptr->DISPLAY_WIDTH-1;
 if (x1<0) x1=0;
 if (x2>=iDisplay_Ptr->DISPLAY_HEIGHT) x2=iDisplay_Ptr->DISPLAY_HEIGHT-1;
 
 for(int32_t y=y1;y<=y2;y++)
 {
  for(int32_t x=x1;x<=x2;x++) 
	{
	 if (x>=0 && x<iDisplay_Ptr->DISPLAY_HEIGHT && y>=0 && y<iDisplay_Ptr->DISPLAY_WIDTH) iDisplay_Ptr->PutPixel(iDisplay_Ptr->DISPLAY_WIDTH-1-y,x,color);
	}
 }
}

//----------------------------------------------------------------------------------------------------
//���������� ����������� �����������
//----------------------------------------------------------------------------------------------------
void CDisplayStandardLibrary::FillTriangle(int32_t ax,int32_t ay,int32_t bx,int32_t by,int32_t cx,int32_t cy,uint16_t color)
{
 int32_t tmp_x;
 int32_t tmp_y;	
	
 if (ay>cy)
 {  	 
  tmp_x=ax;
	tmp_y=ay;
  ax=cx;
	ay=cy;
  cx=tmp_x;
  cy=tmp_y;
 }
 if (ay>by)
 {
  tmp_x=ax;
	tmp_y=ay;
  ax=bx;
	ay=by;
  bx=tmp_x;
  by=tmp_y;
 }
 if (cy<by)
 {
  tmp_x=cx;
	tmp_y=cy;
  cx=bx;
	cy=by;
  bx=tmp_x;
  by=tmp_y;
 }

 //��������, �������� �� ����������� � ���� ���������
 int32_t starty=ay;
 int32_t endy=cy;
 if (starty==endy) return;//������ ��������
 if (starty>=iDisplay_Ptr->DISPLAY_WIDTH) return;//����������� �� �����
 if (endy<0) return;//����������� �� �����
 if (starty<0) starty=0;
 if (endy>=iDisplay_Ptr->DISPLAY_WIDTH) endy=iDisplay_Ptr->DISPLAY_WIDTH-1;

 //������� ������� � ������ ��������� �� �������� �����
 float lyca=(cy-ay);
 float lyba=(by-ay);
 float offset=starty-ay;
 //x
 float dcx1=(cx-ax)/lyca;
 float dcx2=(bx-ax)/lyba;
 float cx1=ax+offset*dcx1;
 float cx2=ax+offset*dcx2;

 bool first_half=true;

 for(int32_t sy=starty;sy<=endy;sy++,cx1+=dcx1,cx2+=dcx2)
 {
  float x1,x2;
  float z1,z2;

  x1=cx1;

  if (sy<by)
  {
   x2=cx2;
  }
  else
  {
   if (cy==by)
   {
    x2=bx;
   }
   else
   {
    if (first_half==true)
    {
     float lycb=(cy-by);
     float offset=sy-by;

     dcx2=(cx-bx)/lycb;
     cx2=bx+offset*dcx2;
			
     first_half=false;
    }
    x2=cx2;
   }
  }
  if (x1>x2)
  {
   float tmp=x1;
   x1=x2;
   x2=tmp;
   tmp=z1;
   z1=z2;
   z2=tmp;
  }
  //������ ����� ������������
  DrawTriangleLine(sy,x1,x2,color);
 }
}



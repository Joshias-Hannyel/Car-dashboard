/**
 * @file lcd.h
 * @author Giovani Baratto (Giovani.Baratto@ufsm.br)
 * @brief
 * @version 0.1
 * @date 2022-06-17
 *
 * @copyright Copyright (c) 2022
 *
 */

/* ligar
        RS      E       D7      D6      D5      D4
        PB5     PB4     PB3     PB2     PB1     PB0   <- portas
        11      10      50      51      52      53    <- terminais do ATMega 2560

  ligar
        VSS - terra
        VDD - 5V
        RW  - terra
        A - resistor (220 ohms) e depois +5V
        K - terra
*/

#ifndef _LCD_H_
#define _LCD_H_

#include <stdint.h>

#define RS PB5
#define EN PB4

#define COMANDO 0
#define DADO 1

void LCD_pulso_EN(void);
void LCD_escreve(uint8_t rs, uint8_t dado);
void LCD_Init(void);
void LCD_string(const char *str);
void LCD_string_flash(const char *str);
void LCD_gotoxy(uint8_t linha, uint8_t coluna);
void LCD_clear(void);
void LCD_adiciona_caractere(uint8_t numero, uint8_t *dado);

#endif //_LCD_H_

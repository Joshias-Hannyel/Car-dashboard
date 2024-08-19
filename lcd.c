/**
 * @file lcd.c
 * @author Giovani Baratto (Giovani.Baratto@ufsm.br)
 * @brief
 * @version 0.1
 * @date 2022-06-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#define F_CPU 16000000UL
#include "lcd.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/**
 * @brief Gera um pulso em E
 * 
 */
void LCD_pulso_EN(void)
{
    PORTB |= (1 << EN);
    _delay_us(1);
    PORTB &= ~(1 << EN);
}

/**
 * @brief Escreve um dado ou um comando no mostrador LCD, no modo de 4 bits
 * 
 * @param rs 0 = comando ou 1 = dado. Usamos as definições COMANDO ou DADO
 * @param dado dado que será enviado para o mostrador LCD
 */
void LCD_escreve(uint8_t rs, uint8_t dado)
{
    PORTB = (PORTB & 0xF0) | (dado >> 4);
    PORTB = (rs == 0) ? PORTB & ~(1 << RS) : PORTB | (1 << RS);
    LCD_pulso_EN();
    _delay_us(1);

    PORTB = (PORTB & 0xF0) | (dado & 0x0F);
    LCD_pulso_EN();
    /* ver https://www.electronicwings.com/avr-atmega/lcd16x2-interfacing-with-atmega16-32*/
    if (rs == DADO)
    {
        _delay_ms(1);
    }
    else
    {
        _delay_ms(3);
    }
}

/**
 * @brief Inicializa o mostrador LCD no modo de 4 bits
 * 
 */
void LCD_Init(void)
{
    DDRB = 0x7F;              // ATENÇÃO 
    _delay_ms(20);

    LCD_escreve(COMANDO, 0x02); /* inicialização do mostrador LCD no modo de 4 bits */
    LCD_escreve(COMANDO, 0x28); /* 2 linhas e caracteres no formato 5*8, no modo de 4 bits */
    LCD_escreve(COMANDO, 0x0C); /* mostrador ligado, cursor desligado */
    LCD_escreve(COMANDO, 0x06); /* desloca o cursor para a direita a cada dado gravado */
    LCD_escreve(COMANDO, 0x01); /* apaga o mostrador LCD*/
}

/**
 * @brief Escreve uma string da memória RAM para o mostrador LCD
 * 
 * @param str string (na memória RAM) que será escrita
 */
void LCD_string(const char *str)
{
    uint8_t ch;
    while (ch = *str++)
    {
        LCD_escreve(DADO, ch);
    }
}

/**
 * @brief Escreve uma string da memória FLASH para o mostrador LCD
 * 
 * @param str string (na memória FLASH) que será escrita
 */
void LCD_string_flash(const char *str)
{
    uint8_t ch;
    while (ch = pgm_read_byte(str++))
    {
        LCD_escreve(DADO, ch);
    }
}

/**
 * @brief Posiciona o cursor em uma coluna de uma linha
 * 
 * @param linha linha onde será posicionado o cursor. A primeira linha é 0.
 * @param coluna  coluna onde será posicionado o cursor. A primeira coluna é 0.
 */
void LCD_gotoxy(uint8_t linha, uint8_t coluna)
{
    uint8_t add;
    switch (linha)
    {
    case 0:
        add = coluna;
        break;
    case 1:
        add = coluna + 0x40;
        break;
    default:
        return;
    }
    LCD_escreve(COMANDO, 0x80 | add);
}

/**
 * @brief Apaga o mostrador LCD
 * 
 */
void LCD_Clear(void)
{
    LCD_escreve(COMANDO, 0x01); /* Clear display */
}


/**
 * @brief Adiciona um caractere a memória RAM do gerador de caracteres. Os caracteres são armazenados nos primeiros 8 códigos do gerador
 * 
 * @param numero código (valor entre 0 e 7) atribuido ao caractere do usuário
 * @param dado caractere do usuário (veja página 19 do manual)
 */
void LCD_adiciona_caractere(uint8_t numero, uint8_t *dado)
{
    LCD_escreve(COMANDO, 0x40 | (numero << 3));
    for (uint8_t i = 0; i < 8; i++, dado++)
    {
        LCD_escreve(DADO, *dado);
    }
}

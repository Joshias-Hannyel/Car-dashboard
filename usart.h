#ifndef _USART_H_
#define _USART_H_

/**
 * @file usart.h
 * @author Giovani Baratto (Giovani.Baratto@ufsm.br)
 * @brief Procedimentos para configurar e usar a USART0. A interface USART0 é 
 * configurada para receber e transmitir quadros no formato 8N1 a 57600 Bd. Foram
 * escritos procedimentos para transmitir e receber um byte, para transmitir uma
 * string da memória FLASH e para transmitir da memória RAM.
 * @version 0.2
 * @date 2023-05-08
 */

#include <stdint.h>

/**
   @brief Configura o periférico USART no modo assíncrono e quadros no formato 8N1. A taxa de transmissão é 57600 BAUD,
   considerando uma frequência de relógio de 16 MHz. Habilita a USART para receber e transmitir (full-duplex). Veja 
   página 206 do manual de especificações do ATMega2560
*/
void USART0_configura(void);

/**
 * @brief Transmite um byte (dado) pela USART0. Veja a página 207 do manual de especificações do ATmega2560
 * @param dado O byte qye será transmitido.
 */
void USART0_transmite(uint8_t dado);

/**
 * @brief Recebe um byte da USART0
 * 
 * @return uint8_t. Byte recebido da USART0. Veja a página 210 do manual de 
 * especificações do ATMega2560.
 */
uint8_t USART0_recebe(void);

/**
 * @brief Transmite pela USART os bytes da string str, armazenada na memória SRAM
 * @param str Ponteiro para uma string armazenada na memória RAM
 */
void USART0_transmite_string_RAM(uint8_t *str);

/**
 * @brief Transmite pela USART os bytes da string str, armazenada na memória FLASH
 * @param str Ponteiro para uma string armazenada na memória FLASH
 */
void USART0_transmite_string_FLASH(uint8_t *str);

#endif _USART_H_

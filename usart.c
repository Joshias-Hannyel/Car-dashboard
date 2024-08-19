#include "usart.h" /* incluímos o arquivo cabeçalho com o protótipo dos procedimentos para a USART0 */

#define __AVR_ATmega2560__ /* programa para o microcontrolador ATmega2560 */
#ifndef F_CPU /* se não foi definida a frequência de relógio do microcontrolador */
#define F_CPU 16000000UL /* definimos a frequência de relógio da placa Arduino Mega: 16 MHz */
#endif /* F_CPU */
#include <avr/io.h> /* definições das entradas e saídas */
#include <avr/pgmspace.h> /* usamos procedimentos que acessam a memória FLASH: pgm_read_byte()*/

void USART0_configura(void)
{
    UBRR0 = 0x022; //UBRR0H = 0 e UBRR0H = 0x22
    UCSR0A = 0x02; // RXC0=0, TXC0=0, UDRE0=0, FE0=0, DOR0=0, UPE0=0, U2X0=1 e MPCM0 = 0
    UCSR0B = 0x18; // RXCIE0=0,  TXCIE0=0,  UDRIE0=0,  RXEN0=1,  TXEN0=1, UCSZ0[2]=0, RXB80=0 e TXB80=0
    UCSR0C = 0x06; //UMSEL0[1:0]=00, UPM0[1:0]=00, USBS0=0, UCSZ0[1:0]=11 e UCPOL0=0
}

void USART0_transmite(uint8_t dado)
{

    while (!(UCSR0A & (1 << UDRE0))) // esperamos o buffer de transmissão ficar vazio
        ;
    UDR0 = dado; // colocamos o dado a ser transmitido no buffer de transmissão
}

uint8_t USART0_recebe(void)
{

    while (!(UCSR0A & (1 << RXC0))) // espera o dado ser recebido
        ;
    return UDR0; // retorna o dado recebido no buffer de recepção
}

void USART0_transmite_string_RAM(uint8_t *str)
{
    uint8_t ch;
    // Este laço while é executado até encontrarmos o caractere ch = 0x00, marcando o final da string
    while (ch = *str++) // carregamos um caractere da string e apontamos para o próximo caractere
    {
        USART0_transmite(ch); // transmite o caractere pelo terminal TX da USART0
    }
}

void USART0_transmite_string_FLASH(uint8_t *str)
{
    uint8_t ch;
    // Este laço while é executado até que o caractere ch = 0x00, marcando o final da string.
    // Os caracteres são lidos da memória FLASH.
    while (ch = pgm_read_byte(str++)) // carregamos um caractere da string e apontamos para o próximo caractere
    {
        USART0_transmite(ch); // transmite o caractere pelo terminal TX da USART0
    }
}

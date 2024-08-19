//////////////////////////////////////////////////////////////////////////////////////
/*
ANÁLISE E PROJETO DE SISTEMAS LÓGICOS PROGRAMÁVEIS - ELC1119
Diogo Kist Poersch E Joshias Hannyel Garcia
PROJETO: PAINEL PARA UM CARRO
*/
//////////////////////////////////////////////////////////////////////////////////////
/*
Consierarmos um veículo com roda de diâmetro de 19 polegadas, um raio de aproximadamente 24 cm.
O que nos da uma c = 2*pi*(24*10**-2) =  1.508 m
    > Botão preto: simula o pulso da roda do carro.
*mostra a distância percorrida desde que o programa foi iniciado. A velocidade é a velocidade
média a cada três segundos. Ou seja, para fim de testes, se pressionamos o botão uma vez a 
velocidade indicada deve ser 1.508/3 = (aproximadamente) 0.502 m/s ou 1.81 km/h e a distância deve
aumentar em 1.508 metros a cada vez que pressionamos o botão, tendo em vista que a circunferência
da roda é de 1.508 metros.
*/
//  > LED branco representa os faróis
/*
Sistema de circulação de ar vai ser representado por um servomotor.
    > Botão amarelo liga e desliga ele.
*/
/*
Sensor de temperatura indica a temperatura de 3 em 3 segundos via USART e mostra no lcd.
*/
/*
AC: três botões, um que liga ou desliga (cinza), um para baixar (azul) e outro para aumentar (vermelho). Se o valor do AC estiver estiver acima da
temperatura (dada pelo sensor) ligamos o led que esquenta (representado pelo LED vermelho) e se
estiver abaixo ligamos o sistema que resfria (representado pelo led azul).
*/
//  > LED vermelhor indica que o AC está esquentando
//  > LED azul indica que o AC está resfriando
//  > Botão branco: para controle do que está mostrando no lcd

#define __AVR_ATmega2560__
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "usart.h"
#include "lcd.h"


/////////////////////////////////////////////////////////////////////////////
#define TH_MIN 1000        // para tH = 500 uS, 0º
#define TH_MAX 4800        // para tH = 2400 uS, 180º

void configura_TC1_PWM_modo_14(void)
{
  TCCR1A = 0x22;       // COM1A[1:0]=00, COM1B[1:0]=10,COM1C[1:0]=00, WGM1[1:0]=10
  TCCR1B = 0x1A;       // ICNC1=0, ICES1=0, WGM1[3:2]=11, CS1[2:0]=010
  TCCR1C = 0x00;       // FOC1A=0, FOC1B=0
  ICR1 = 39999;        // TPWM=20ms, para N=8 e fSYS = 16 MHz
  OCR1B = 0;
  DDRB |= (1 << DDB6); // fazemos OC1B uma saída (PB6)
}
//////////////////////////////////////////////////////////////////////////////////

void configura_TIMER0(void) {
  /* configura o timer0 para gerar interrupções a cada 1 ms e executar a rotina
    de interrupção TIMER0_COMPA_vect */
  TCCR0A = 0x02; // saídas OC0A e OC0B como portas normais. COM0A[1:0]=00 COM0B[1:0]=00 WGM0[2:0]=010
  TCCR0B = 0x03; // dividimos a frequência de relógio por 64. FOC0A = 0 FOC0B = 0 WGM0[2:0]=010 CS0[2:0] = 011
  TIMSK0 = 0x02; // interrupções ocorrem no overflow do contador. OCIE0B = 0 OCIE0A = 1 TOIE0 = 0
  OCR0A = 0xF9;  // valor de TOP = 249 = 0xF9
  TCNT0 = 0x00;  // zeramos o contador
}

/////////////////////////////////////////////////////////////////////////////////
// conversao A/D para sensor de temperatura.

/*
configura a os registradores do conversor A/D para atuar com frequencia de 125 KHz, 
ADPS[2:0]=111, a tensao de referencia é AVCC (5V), REFS[1:0]=01, usaremos a porta 
ADC5, MUX[5:0]=000101. Para habilitar o conversor, fazemos ADEN=1.
*/
void ADC_configura(void)
{
    ADCSRA = 0x87; // ADEN=1 e ADPS[2:0]=111
    ADCSRB = 0x00; // MUX[5]=0
    ADMUX = 0x45;  // REFS[1:0]=01 e  MUX[4:0]=00101
}

// função para leitura do valor no conversor A/D:
uint16_t ADC_medida(void)
{
    ADCSRA |= (1 << ADSC);       // fazendo ADSC=1 em ADCSRA, iniciamos uma nova conversão
    while (ADCSRA & (1 << ADSC)) // enquanto ADSC=1, a conversão está em andamento
        ;
    return ADC; // retorna o valor do canal convertido
}

// função para transmitir a temperatura pela USART0:
volatile float temperatura; // variavel global.
void transmite_ADC_temp(uint16_t leitura, float vref)
{
    const float BETA = 3950; // depende do thermistor.
    float temp;

    USART0_transmite_string_FLASH(PSTR("\nTemperatura: "));
    // a equação usada é dada pelo proprio wokwi.
    temp = 1 / (log(1 / (1023. / leitura - 1)) / BETA + 1.0 / 298.15) - 273.15;
    temperatura = temp;
    USART0_transmite_float(temp);
    USART0_transmite_string_FLASH(PSTR(" °C"));
    //USART0_transmite_string_FLASH(PSTR("\r\n"));
}
//////////////////////////////////////////////////////////////////////////////////////

void configPortas(void) {
  // Botão que simula o sensor da roda SEM BOUNCE
  DDRC &= (~(1<<DDC6)); // fizemos a porta 6 ser de entrada || porta pc6 = 31
  PORTC |= (1<<PORTC6); // colocamos resistor interno

  // sensor de luz
  DDRC &= (~(1<<DDC7));  // porta de entrada                || porta pc7 = 30
  PORTC |= (1<<PORTC7);  // colocamos resistor interno
  DDRD |= (1<<DDD0);     // farois = led branco; saída      || porta pd0 = 21
  PORTD &= (~(1<<PORTD0));// começa apagado.

  // botão circulação (amarelo)
  DDRD &= (~(1<<DDD7)); // porta de entrada                 || porta pd7 = 38
  PORTD |= (1<<PORTD7);   // resistor interno

  // AC
  DDRC &= (~(1<<DDC0));     // porta de entrada             || porta pc0 = 37 (+)
  PORTC |= (1<<PORTC0);     // resistor interno
  DDRC &= (~(1<<DDC1));     // porta de entrada             || porta pc1 = 36 (on/off)
  PORTC |= (1<<PORTC1);     // resistor interno
  DDRC &= (~(1<<DDC2));     // porta de entrada             || porta pc2 = 35 (-)
  PORTC |= (1<<PORTC2);     // resistor interno
  // leds AC:
  DDRC |= (1<<DDC3);        // led azul, resfriar; saída    || porta pc3 = 34
  PORTC &= (~(1<<PORTC3));  // começa apagado.
  DDRC |= (1<<DDC4);        // led vermelho, aquece; saída  || porta pc4 = 33
  PORTC &= (~(1<<PORTC4));  // começa apagado.

  // controle lcd
  DDRC &= (~(1<<DDC5));     // controle display             || porta pc5 = 32
  PORTC |= (1<<PORTC5);
}

// funcao para transmitir um float pela USART0:
void USART0_transmite_float(float dado) {
    char buffer[11];
    dtostre(dado, buffer, 3, DTOSTR_UPPERCASE);
    USART0_transmite_string_RAM(buffer);
}


volatile float distancia=0, t_volta=0, velocidade=0;

volatile uint16_t t0=1950, t2=0;
volatile uint8_t atualiza=0, t1, vent=0;

uint8_t grau[] = {0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00};


ISR(TIMER0_COMPA_vect) {
  t1++;
  if (t1 == 15) {
    t_volta += 0.015;
    t1 = 0;
  }

  t0++;
  if (t0 == 3000) {
    // atualiza a cada 3 seundos
    atualiza = 1;
    t0=0;
  }

  t2++;
  if (t2 == 500) {
    vent++;
    t2 = 0;
    if (vent == 4) vent = 0;
  }
}

int main (void) {
  sei(); //habilita interrupções
  LCD_Init();
  USART0_configura();
  configura_TIMER0();
  configura_TC1_PWM_modo_14();
  ADC_configura();
  configPortas();
  
  LCD_adiciona_caractere(0, grau);
  

  USART0_transmite_string_FLASH(PSTR("Programa iniciando..."));

  //botão sensor da roda
  uint8_t leitura0, leitura1, chave, voltas=0;
  leitura0 = PINC & (1<<PINC6); // 1 = chave aberta; 0 = chave fechada
  

  // sensor de luz
  uint8_t farois0;


  // circulacao
  uint8_t A_nowState=0, A_prevState, A_read0, A_read1, circulacao=0; // botao amarelo
  A_read1 = (PIND & (1 << PIND7));           	// leitura inicial
  A_nowState = A_read1;				                // estado inicial


  //temperatura
  uint16_t read_t;


  //AC
  uint8_t R_nowState=0, R_prevState, R_read0, R_read1; // botao red
  R_read1 = (PINC & (1 << PINC0));           	// leitura inicial
  R_nowState = R_read1;				                // estado inicial

  uint8_t G_nowState=0, G_prevState, G_read0, G_read1, AC=0; // botao grey
  G_read1 = (PINC & (1 << PINC0));           	// leitura inicial
  G_nowState = G_read1;				                // estado inicial

  uint8_t B_nowState=0, B_prevState, B_read0, B_read1; // botao blue
  B_read1 = (PINC & (1 << PINC0));           	// leitura inicial
  B_nowState = B_read1;				                // estado inicial

  float AC_temp=18.0;


  //lcd
  uint8_t W_nowState=0, W_prevState, W_read0, W_read1, display=0; // white button
  W_read1 = (PINC & (1 << PINC5));           	// leitura inicial
  W_nowState = W_read1;				                // estado inicial
  char buffer[10];
  

// 20ms de atraso permite velocidade máxima de medição = 270 km/h
	while(1) {
    _delay_ms(10); // intervalo maior que o repique

    // atualiza valores
    if (atualiza) {
      USART0_transmite_string_FLASH(PSTR("\n\n"));

      // velocidade e distância
      USART0_transmite_string_FLASH(PSTR("Velocidade: "));
      USART0_transmite_float(velocidade);
      USART0_transmite_string_FLASH(PSTR(" Km/h"));
      USART0_transmite_string_FLASH(PSTR("\nDistancia: "));
      USART0_transmite_float(distancia);
      USART0_transmite_string_FLASH(PSTR(" metros"));
      t_volta = 0;
      voltas = 0;

      //farois
      USART0_transmite_string_FLASH(PSTR("\nStatus dos farois: "));
      if(farois0) USART0_transmite_string_FLASH(PSTR("ACESO"));
      else USART0_transmite_string_FLASH(PSTR("DESLIGADO"));

      // circulação
      USART0_transmite_string_FLASH(PSTR("\nStatus da circulação: "));
      if (circulacao) USART0_transmite_string_FLASH(PSTR("LIGADO"));
      else USART0_transmite_string_FLASH(PSTR("DESLIGADO"));

      // temperatura
      read_t = ADC_medida();
      transmite_ADC_temp(read_t, 5.0);

      //AC
      if(AC) {
        USART0_transmite_string_FLASH(PSTR("\nTemperatura do AC: "));
        USART0_transmite_float(AC_temp);
      }

      LCD_Clear();
      if (display) {
        // mostrar temperatura, temp AC e se a circulação está ativa.
        LCD_gotoxy(0, 0);
        LCD_string_flash(PSTR("Temp. "));
        dtostrf(temperatura, 4, 1, buffer);   // float -> string
        LCD_string(buffer);
        LCD_escreve(DADO, 0);
        LCD_string_flash(PSTR("C"));
        LCD_gotoxy(1,9);
        LCD_string_flash(PSTR("Circ: "));
        if (circulacao) LCD_string_flash(PSTR("1"));
        else LCD_string_flash(PSTR("0"));
        if (AC) {
          LCD_gotoxy(1,0);
          dtostrf(AC_temp, 2, 0, buffer);
          LCD_string(buffer);
          LCD_escreve(DADO, 0);
          LCD_string_flash(PSTR("C"));
        }
        
      } else {
        LCD_gotoxy(0, 0);
        LCD_string_flash(PSTR("V = "));
        dtostrf(velocidade, 5, 2, buffer);
        LCD_string(buffer);
        LCD_string_flash(PSTR(" Km/h"));

        LCD_gotoxy(1, 0);
        LCD_string_flash(PSTR("Dist: "));
        dtostrf(distancia, 4, 1, buffer);
        LCD_string(buffer);
        LCD_string_flash(PSTR(" m"));
        if (farois0){
          LCD_gotoxy(1, 14);
          LCD_string_flash(PSTR("><"));
        }
      }

      atualiza = 0;
    }

		// calculo da velocidade e distância
    leitura1 = PINC & (1<<PINC6);
    if ((!leitura0 == 0) && (leitura1 == 0)) chave = 1;
    else chave = 0;
    leitura0 = leitura1;
    if (chave) {
      voltas++; // giros da roda, cada giro complementa 1. Carro percorreu 1.508 metros
      distancia += 1.508; //metros
    }
    if (voltas>0) velocidade = 3.6 * (voltas*1.508)/(t_volta); // km/h 
    else velocidade = 0;


    // farois
    farois0 = PINC & (1<<PINC7);
    if (farois0) {
      //led asceso
      PORTD |= (1<<PORTD0);
    } else {
      //led apagado
      PORTD &= (~(1<<PORTD0));
    }


    // circulação
    A_read0 = A_read1; 				                  // att leitura 0
    A_read1 = (PIND & (1 << PIND7));           	// leitura atual 
    A_prevState = A_nowState;			              // att estado antigo
    if (A_read1 == A_read0) {			              // se sinal permanecer estavel
	    A_nowState = A_read1;                     // alteramos o estado para a leitura atual
    }
    if ( (!A_prevState == 0) && (A_nowState == 0) ) { // so é verdadeiro quando o botão é pressionado
	    circulacao ^= 1;
    }
    if (circulacao) {
      if (vent == 1) OCR1B = TH_MIN;
      if (vent == 3) OCR1B = TH_MAX;
    }


   // AC
    G_read0 = G_read1; 				                  // att leitura 0
    G_read1 = (PINC & (1 << PINC1));           	// leitura atual
    G_prevState = G_nowState;			              // att estado antigo
    if (G_read1 == G_read0) {			              // se sinal permanecer estavel
	    G_nowState = G_read1;                     // alteramos o estado para a leitura atual
    }
    if ( (!G_prevState == 0) && (G_nowState == 0) ) { // so é verdadeiro quando o botão é pressionado
	    AC ^= 1;
    }
    if (AC) {
      // so entra quando ac on
      if (AC_temp>=16. && AC_temp<=30.) {
        R_read0 = R_read1; 				                  // att leitura 0
        R_read1 = (PINC & (1 << PINC0));           	// leitura atual
        R_prevState = R_nowState;			              // att estado antigo
        if (R_read1 == R_read0) {			              // se sinal permanecer estavel
	        R_nowState = R_read1;                     // alteramos o estado para a leitura atual
        }
        if ( (!R_prevState == 0) && (R_nowState == 0) ) { // so é verdadeiro quando o botão é pressionado
	        if (AC_temp != 30.) AC_temp++;
        }
        B_read0 = B_read1; 				                  // att leitura 0
        B_read1 = (PINC & (1 << PINC2));           	// leitura atual
        B_prevState = B_nowState;			              // att estado antigo
        if (B_read1 == B_read0) {			              // se sinal permanecer estavel
	        B_nowState = B_read1;                     // alteramos o estado para a leitura atual
        }
        if ( (!B_prevState == 0) && (B_nowState == 0) ) { // so é verdadeiro quando o botão é pressionado
	        if (AC_temp != 16.) AC_temp--;
        }

        if(AC_temp > temperatura) {
          PORTC |= (1<<PORTC4);     // red on
          PORTC &= (~(1<<PORTC3));  // blue off
        } else {
          PORTC |= (1<<PORTC3);     // blue on
          PORTC &= (~(1<<PORTC4));  // red off
        }
      }
    } else {
      // AC off, desligar os leds
      PORTC &= (~(1<<PORTC4)); // red off
      PORTC &= (~(1<<PORTC3)); // blue off
    }

    // lcd
    W_read0 = W_read1;
    W_read1 = (PINC & (1 << PINC5));
    W_prevState = W_nowState;
    if (W_read1 == W_read0) {
	    W_nowState = W_read1;
    }
    if ( (!W_prevState == 0) && (W_nowState == 0) ) {
	    display ^= 1;
    }
	}
}

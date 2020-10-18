#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "def_principais.h"
#include "LCD.h"
#include "USART.h"

//Propriedades
volatile unsigned char Umidade = 0;
volatile unsigned char Temp = 0;
volatile unsigned char ResLDR = 0;
volatile unsigned char Dist = 0;

//Utilitários
volatile float Div = 0;
volatile unsigned char Contador = 0;
volatile unsigned char Sinal = 0;

void Terminal(unsigned char i);

int main()
{

    DDRB = 0b11111110;
    PORTB = 0b00000111;

    DDRD = 0b11111000;
    PORTD = 0b00101111;

    unsigned char i;

    //Configuracao do Conversor Analogico Digital
    ADMUX = 0b01000000;

    ADCSRA = 0b11001111;
    ADCSRB = 0b00000000;

    DIDR0 = 0b00000111;

    //Configuracao do PWM
        //Temporizador 1
        TCCR1A = (1<<WGM11)|(1<<COM1B1);
        TCCR1B = (1<<CS11) | (1<<WGM13) | (1<<WGM12);

        ICR1 = 3999;
        OCR1B = 0;

        //Temporizador 2
        TCCR2A = (1<<COM2A1) | (1<<WGM21) | (1<<WGM20) | (1<<COM2B1);
        TCCR2B = (1<<CS20)|(1<<CS21);

        //Saidas
        OCR2A = 0;
        OCR2B = 0;

    //Configuracao do Sensor Ultrassonico
        //Temporizador 0
        TIMSK0=(1<<TOIE0);

        //Interrupcoes
            //Interrupcao PB0
            PCICR = (1<<PCIE0);
            PCMSK0 = (1<<PCINT0);

            //Interrupcao PD2
            PCICR |= (1<<PCIE2);
            PCMSK2 = (1<<PCINT18);


    inic_LCD_4bits();
    _delay_ms(500);

    cmd_LCD(0x80,0);
	escreve_LCD("T:");

	cmd_LCD(0x89,0);
	escreve_LCD("U:");

	cmd_LCD(0xC0,0);
	escreve_LCD("L:");

	cmd_LCD(0xC9,0);
	escreve_LCD("D:");

    USART_Inic(MYUBRR);

    sei();

    PORTB = PORTB | 0b00000010;
    _delay_us(10);
    PORTB = PORTB & 0b11111101;
    _delay_us(50);

    while(1)
    {
        if(tst_bit(Sinal, 0))
        {
            Sinal&=254;

            if(!tst_bit(ADMUX, 1) && !tst_bit(ADMUX, 0))
            {

                Temp=ADC*0.48876;

                if(Temp<30)
                {
                    OCR2A=0;
                }

                else if(Temp>=30 && Temp<=60)
                {
                    OCR2A=((255*Temp)/60);
                }

                else
                {
                    OCR2A = 255;
                }

                ADMUX = 0b01000001;
            }

            else if(tst_bit(ADMUX, 0))
            {

                Umidade = 100;
                Umidade -= ADC*0.09775;

                if(tst_bit(PIND,PD2) || Umidade >= 50)
                {
                    OCR2B=0;
                }

                else if(Umidade<50)
                {
                    OCR2B=((255*80-255*Umidade)/80);
                }

                ADMUX = 0b01000010;

            }

            else if(tst_bit(ADMUX, 1))
            {
                Div=1024-ADC;
                Div=1023/Div;
                Div*=1000;
                Div*=ADC*0.004888;
                ResLDR=Div;

                if(ResLDR>1250 && !tst_bit(Sinal,1))
                {
                    OCR1B = 2000;
                    Sinal|=2;
                }

                else if(ResLDR>1000 && tst_bit(Sinal,1))
                {
                    if(OCR1B<4000)
                        OCR1B+=200;
                }
                else if(ResLDR<100 && tst_bit(Sinal,1))
                {
                    if(OCR1B>0)
                        OCR1B-=200;
                    else
                    {
                        OCR1B=0;
						Sinal&=253;
                    }
                }

                ADMUX = 0b01000000;

                PORTB = PORTB | 0b00000010;
                _delay_us(10);
                PORTB = PORTB & 0b11111101;
            }
        }
        _delay_ms(250);
        Terminal(i);
        ADCSRA = ADCSRA | 1<<ADSC;
        _delay_ms(250);
    }
}

ISR(PCINT0_vect)
{

    if(tst_bit(PINB,PB0))
    {
        TCCR0B = (1<<CS00)|(1<<CS02);
        TCNT0=0;
    }

    else if(!tst_bit(PINB,PB0))
    {
        Dist=TCNT0;
        Dist = Dist + (255 * Contador);
        Dist *= 1.10344;
        TCCR0B = 0;
        Contador = 0;
    }
}

ISR(PCINT2_vect)
{
    if(tst_bit(PIND,PD2))
    {
        OCR2B=0;
    }
}

ISR(TIMER0_OVF_vect)
{
    Contador++;
}

ISR(ADC_vect)
{
    Sinal|=1;
}

void Terminal(unsigned char i)
{
    escreve_USART("Temperatura:");
    numero_USART(Temp);
    escreve_USART("\r");

    escreve_USART("Umidade:");
    numero_USART(Umidade);
    escreve_USART("\r");

    escreve_USART("Luz:");
    numero_USART(ResLDR);
    escreve_USART("\r");

    escreve_USART("Distancia:");
    numero_USART(Dist);
    escreve_USART("\r");

    for(i=0;i<5;i++){
		//limpa os caracteres depois de 0x8C e antes de 0x8F
		cmd_LCD(0x82+i,0);//posiciona cursor
		cmd_LCD(1,1);//limpa endereço
		//limpa os caracteres depois de 0x83 e antes de 0x88
		cmd_LCD(0x8B+i,0);//posiciona cursor
		cmd_LCD(1,1);//limpa endereço
		//limpa os caracteres depois de 0xC3 e antes de 0xC8
		cmd_LCD(0xC2+i,0);//posiciona cursor
		cmd_LCD(1,1);//limpa endereço
		//limpa os caracteres depois de 0xCC e antes de 0xCF
		cmd_LCD(0xCB+i,0);//posiciona cursor
		cmd_LCD(1,1);//limpa endereço
	}

    print_LCD(Temp, 0x82);
    escreve_LCD("C");

    print_LCD(Umidade, 0x8B);
    escreve_LCD("%");

    print_LCD(ResLDR, 0xC2);

    print_LCD(Dist, 0xCB);
    escreve_LCD("cm");

}

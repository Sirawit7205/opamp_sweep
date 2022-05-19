#include <stdio.h>
#include <stdlib.h>

#define F_CPU 4000000UL

#define TOP_LOWER_BOUND   99
#define TOP_UPPER_BOUND   199

#define SAMPLING_START    20

volatile uint16_t current_top;
volatile uint16_t adc_results[TOP_UPPER_BOUND - TOP_LOWER_BOUND + 1] = {};
volatile uint8_t compare_match_count;
volatile bool request_freq_change;

char print_buffer[20];
bool ended = true;

ISR(TIMER1_COMPA_vect) {
  if(request_freq_change) {
    request_freq_change = false;
    compare_match_count = 0;
    current_top--;
  }

  if(compare_match_count++ == SAMPLING_START) {
    compare_match_count = 0;
    ADCSRA |= _BV(ADSC);
  }
  
  if(current_top < TOP_LOWER_BOUND) {
    TCCR1B &= ~_BV(CS10);
  } else {
    OCR1A = current_top;
  }
}

ISR(ADC_vect) {
  adc_results[current_top - TOP_LOWER_BOUND] = ADC;
  request_freq_change = true;
}

void send_char(char c) {
  while(!(UCSR0A & _BV(UDRE0)));
  UDR0 = c;
}

void send_string(char *string, int len) {
  for(int i = 0; i < len; i++) {
    send_char(string[i]);
  }
}

void setup() {
  DDRD &= ~_BV(DDD4);     //set port PD4 to input
  PORTD |= _BV(PORTD4);   //enable internal pullup on port PD4
  
  CLKPR = _BV(CLKPCE);    //enable prescaler change
  CLKPR = _BV(CLKPS1);    //set prescaler to 4 -> core clk is now 4MHz

  UCSR0A |= _BV(U2X0);                  //double USART speed
  UCSR0B |= _BV(RXEN0) | _BV(TXEN0);    //enable USART0 RX/TX
  UBRR0 = 12;                           //set baud rate to 38400 (with 0.2% error)

  TCCR1A = 0x00;                                      //clear what Arduino put in
  TCCR1B = 0x00;
  TCCR1A |= _BV(COM1A0) | _BV(WGM11) | _BV(WGM10);    //toggle OC1A on compare A match (gives 50% duty cycle)
  TCCR1B |= _BV(WGM13) | _BV(WGM12);                  //fast PWM mode, set freq with OCR1A register, clock not connected yet
  TIMSK1 |= _BV(OCIE1A);                              //interrupt on both compare A match                          
  DDRB |= _BV(DDB1);                                  //set OC1A pin (PB1) to output

  ADMUX |= _BV(REFS0);                                //using AVcc as AREF (and use ADC input 0)
  ADCSRA |= _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0);   //emable ADC in auto-trigger mode, interrupt enabled, set prescaler to 32
  ADCSRB |= _BV(ADTS2) | _BV(ADTS0);                  //using Timer1 compare B match as trigger source
  DIDR0 |= _BV(ADC0D);                                //disable digital input on pin ADC0 (PC0)
  
  sei();
}

void loop() {
  if(!(PIND & _BV(PIND4))) {
    _delay_ms(10);                    //debounce. not actually 10ms due to Arduino F_CPU flag but it's ok.
    ended = false;                    //reset flags
    compare_match_count = 0;
    request_freq_change = false;
    current_top = TOP_UPPER_BOUND;    //begin sweep at 10kHz
    OCR1A = current_top;
    TCNT1 = 0x0000;                   //reset count
    TCCR1B |= _BV(CS10);              //connect clock to Timer1
  }
  
  if(!(TCCR1B & _BV(CS10)) && !ended) {
    ended = true;

    //send header
    uint8_t len = sprintf(print_buffer, "FREQ,ADC\n");
    send_string(print_buffer, len);

    //send data
    for(int i = 0; i < TOP_UPPER_BOUND - TOP_LOWER_BOUND + 1; i++) {
        double freq = ((double)F_CPU / (i + TOP_LOWER_BOUND + 1) / 2);
        double adc = ((double)adc_results[i] / 1023.0) * 5.0;
        char freq_string[10];
        char adc_string[10];
        dtostrf(freq, 8, 2, freq_string);
        dtostrf(adc, 4, 2, adc_string);
        uint8_t len = sprintf(print_buffer, "%s,%s\n", freq_string, adc_string);
        send_string(print_buffer, len);
    }
  }
}

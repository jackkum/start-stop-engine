#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

volatile uint8_t mode = 0x00;
volatile uint8_t wakeup = 0x00;
void trigger(void);
uint8_t isPushed(uint8_t l);

void init(void){
    DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
    DDRD &= ~(1 << DDD4);     // Clear the PD4 pin
    DDRD |=  (1 << DDD3);     // Clear the PD3 pin
    DDRD |=  (1 << DDD5);     // Clear the PD5 pin
    DDRD |=  (1 << DDD6);     // Clear the PD6 pin
    DDRD |=  (1 << DDD7);     // Clear the PD7 pin

    // PD2 is now an input with pull-up enabled
    PORTD |= (1 << PD2);    // turn On the Pull-up
    PORTD |= (1 << PD4);    // turn On the Pull-up
    PORTD |= (1 << PD3);
    PORTD &= ~(1 << PD5);    // turn Off
    PORTD &= ~(1 << PD6);    // turn Off
    PORTD &= ~(1 << PD7);    // turn Off

    EICRA |= (1 << ISC00)|(1 << ISC01);    // set INT0 to trigger on ANY logic change
    EIMSK |= (1 << INT0);     // Turns on INT0

    // enable timer overflow interrupt for both Timer0
    TIMSK0 |= (1<<TOIE0);
    TIFR0  |= (1<<TOV0);

    // set timer0 counter initial value to 0
    TCNT0 = 0x00;
    // start timer0 with /1024 prescaler
    TCCR0B = (1<<CS02) | (1<<CS00);

    // 8.0MHz (8,000,000 / 255 / 1024 = 30.63).
}

void sleep(void){
    // HI-Z
    DDRD  = 0x00;
    DDRC  = 0x00;
    DDRB  = 0x00;
    PORTD = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;

    DDRD  &= ~(1 << DDD2);
    PORTD |= (1 << PD2);

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);

    cli();
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();

    init();
    _delay_ms(100);
}

int main(void) {

    init();

    sei(); // turn on interrupts

    while(1){

        if(mode == 0 && wakeup == 0){
            sleep();
        }

        if(isPushed(PD2)){
            trigger();
            while((PIND & (1<<PD2)) == 0);
        }
    }

    return 0;
}

ISR(INT0_vect){
    //PORTD ^= (1 << PORTD3);
    //PORTD ^= (1 << PD7);
    wakeup = 0xFF;

}

ISR(TIMER0_OVF_vect) {
    if(wakeup > 0){
        wakeup--;
    }
}

uint8_t isPushed(uint8_t l){
    if((PIND & (1<<l)) == 0){
         _delay_ms(50);
        if((PIND & (1<<l)) == 0){
            return 1;
        }
    }

    return 0;
}

void trigger() {

    switch(mode){

        case 3:

            if(isPushed(PD4)){
                mode = 0;
                PORTD &= ~(1 << PD5);    // turn Off
                PORTD &= ~(1 << PD6);    // turn Off
                PORTD &= ~(1 << PD7);    // turn Off
                return;
            }
        break;

        case 2: // On
            if(isPushed(PD4)){
                mode++;
                PORTD |=  (1 << PD5);    // turn On
                PORTD |=  (1 << PD6);    // turn On
                PORTD |=  (1 << PD7);    // turn On
                while((PIND & (1<<PD2)) == 0);
                PORTD &= ~(1 << PD7);    // turn Off
                // set timer, starter work
            } else {
                mode = 0;
                PORTD &= ~(1 << PD5);    // turn Off
                PORTD &= ~(1 << PD6);    // turn Off
                PORTD &= ~(1 << PD7);    // turn Off
                return;
            }

        break;

        case 1: // Acc
            mode++;
            PORTD |=  (1 << PD5);    // turn On
            PORTD |=  (1 << PD6);    // turn On
        break;

        case 0: // off
            mode++;
            PORTD |=  (1 << PD5);    // turn On
            PORTD &= ~(1 << PD6);    // turn Off
        break;

    }
}

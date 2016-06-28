#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>

#define LINE        PORTD
#define ACC         PD5
#define IG          PD6
#define STARTER     PD7

#define MODE_OFF    0
#define MODE_ACC    1
#define MODE_IG     2
#define MODE_START  3
#define MODE_MUFFLE 4

#define ON(b)  do { LINE |=  (1 << b); } while(0)
#define OFF(b) do { LINE &= ~(1 << b); } while(0)

volatile uint8_t mode   = 0x00;
volatile uint8_t wakeup = 0x00;

/**
 * Check button is pushed
 */
uint8_t isPushed(uint8_t l){
    // is pushed
    if((PIND & (1<<l)) == 0){
        // wait
         _delay_ms(50);
        // check again
        if((PIND & (1<<l)) == 0){
            return 1;
        }
    }

    return 0;
}

/**
 * check mode
 */
void trigger(void) {

    switch(mode){

        case MODE_START:

            if(isPushed(PD4)){
                mode = 0;
                OFF(ACC);
                OFF(IG);
                OFF(STARTER);
                return;
            }
        break;

        // rotate key from IG to Starter or off
        case MODE_IG:
            if(isPushed(PD4)){
                ON(ACC);
                ON(IG);
                ON(STARTER);
                while((PIND & (1<<PD2)) == 0);
                OFF(STARTER);

                // TODO: check tahometr sensor the next mode
                mode++;
            } else {
                mode = 0;
                OFF(ACC);
                OFF(IG);
                OFF(STARTER);
                return;
            }

        break;

        // rotate key from ACC to IG
        case MODE_ACC:
            mode++;
            ON(ACC);
            ON(IG);
        break;

        // rotate key from off to ACC
        case MODE_OFF:
            mode++;
            ON(ACC);
            OFF(IG);
        break;

    }
}

/**
 * Initialize IO and timers
 */
void init(void){

    DDRD  = 0xE8; // 11101000
    PORTD = 0x1C; // 00011100

    // set INT0 to trigger on front change
    EICRA  |= (1 << ISC00) | (1 << ISC01);
    // Turns on INT0
    EIMSK  |= (1 << INT0);

    // enable timer overflow interrupt for Timer0
    TIMSK0 |= (1<<TOIE0);
    TIFR0  |= (1<<TOV0);

    // set timer0 counter initial value to 0
    TCNT0  = 0x00;
    // start timer0 with /1024 prescaler
    // 8.0MHz (8,000,000 / 255 / 1024 = 30.63).
    TCCR0B = (1<<CS02) | (1<<CS00);
}

/**
 * go sleep
 */
void sleep(void){

    // all in HI-Z, only PORTD2 is pull-up (button connected)
    DDRD   = 0x00;
    DDRC   = 0x00;
    DDRB   = 0x00;
    PORTD  = 0x04; // 00000100
    PORTB  = 0x00;
    PORTC  = 0x00;

    // set sleep mode
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);

    // disable interrupts
    cli();

    // enable sleep cpu
    sleep_enable();

    // enable interrupts
    sei();

    // sleep cpu
    sleep_cpu();

    // wake up
    sleep_disable();

    // reinit IO and timers
    init();

    // wait
    _delay_ms(100);
}

/**
 * point of entry
 */
int main(void) {

    // init IO and timers
    init();

    // turn on interrupts
    sei();

    // main loop
    while(1){

        // mode off and wake up timeout is empty
        if(mode == 0 && wakeup == 0){
            // sleep mcu
            sleep();
        }

        // check pushed button
        if(isPushed(PD2)){
            // trigger mode change
            trigger();
            // until button is not released
            while((PIND & (1<<PD2)) == 0);
        }
    }

    return 0;
}

/**
 * Interrupt vector: pushed button
 */
ISR(INT0_vect){
    // set 255 ticks
    wakeup = 0xFF;

}

/**
 * Interrupt vector: timer0 overflow
 */
ISR(TIMER0_OVF_vect) {
    // decrement wakeup timeout
    if(wakeup > 0){
        wakeup--;
    }
}


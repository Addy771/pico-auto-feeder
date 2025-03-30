/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "ws2812.pio.h"
#include "Button-debouncer/button_debounce.h"

extern "C"
{
#include "stepper.h"
}

#define WS2812_PIN 16
#define PB_PIN 13
#define LIMIT_SW_PIN 17
#define L293_EN_PIN 22
#define L293_1A 19
#define L293_2A 18
#define L293_3A 21
#define L293_4A 20

#define STEPPER_DELAY_US 2500   // Time in microseconds to wait between motor steps. Lower time value = faster motor movement.
#define MAX_ALARMS 12           // Maximum number of alarms that can be set concurrently

Debounce debouncer;
stepper_t stepper;

datetime_t current_time;
datetime_t feed_alarms[MAX_ALARMS];
uint8_t alarm_count = 0;

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

// Step the specified number of times, unless the end stops are triggered
uint8_t step_limited(stepper_t *s, uint16_t target_steps, stepper_direction_t direction, uint32_t step_delay_us)
{
    for (uint16_t step_cnt = 0; step_cnt < target_steps; step_cnt++)
    {
        // Return with error code if end stop is triggered
        if (debouncer.read(LIMIT_SW_PIN) == 1)
            return 1;

        stepper_step_once(s, direction);
        sleep_us(step_delay_us);
    }

    stepper_release(s); // don't need to use power holding position
    return 0;
}


// Compare two datetimes, return true if the hours, minutes and seconds match, ignoring the rest of the values.
uint8_t is_same_time_hms(datetime_t *timeA, datetime_t *timeB)
{
if ((*timeA).hour == (*timeB).hour && (*timeA).min == (*timeB).min && (*timeA).sec == (*timeB).sec)

        return 1;
    else
        return 0;
}

// Control states
enum
{
    PWRON,
    SLEEP,
    LOAD,
    DISPENSE,
    HOME_OUTER,
    ERROR,
    TEST
};

// Error codes
enum
{
    NONE,
    PWRON_FAIL,
    END_UNEXPECTED,
    HOMING_FAIL
};

int main() 
{
    //set_sys_clock_48();
    stdio_init_all();

    //////// Init IO to safe defaults ////////
    gpio_init(L293_EN_PIN);
    gpio_set_dir(L293_EN_PIN, GPIO_OUT);
    gpio_put(L293_EN_PIN, 0);   // Disable L293 for now

    // Set driver inputs low
    gpio_init(L293_1A);
    gpio_set_dir(L293_1A, GPIO_OUT);
    gpio_put(L293_1A, 0);

    gpio_init(L293_2A);
    gpio_set_dir(L293_2A, GPIO_OUT);
    gpio_put(L293_2A, 0);

    gpio_init(L293_3A);
    gpio_set_dir(L293_3A, GPIO_OUT);
    gpio_put(L293_3A, 0);

    gpio_init(L293_4A);
    gpio_set_dir(L293_4A, GPIO_OUT);
    gpio_put(L293_4A, 0);

    // Limit switch is NC, connected between GPIO and GND
    gpio_init(LIMIT_SW_PIN);
    gpio_set_dir(LIMIT_SW_PIN, GPIO_IN);
    gpio_pull_up(LIMIT_SW_PIN);

    // Pushbutton is NC, connected between GPIO and GND
    gpio_init(PB_PIN);
    gpio_set_dir(PB_PIN, GPIO_IN);
    gpio_pull_up(PB_PIN);    

    current_time = 
    {
        .year = 2024,
        .month = 1,
        .day = 1,
        .dotw = 0,
        .hour = 0,
        .min = 0,
        .sec = 0
    };

    rtc_init();
    rtc_set_datetime(&current_time);

    // use pio0, debouncer should work around this
    PIO pio = pio0;
    int sm = 0;
    pio_sm_claim(pio, sm);  // indicate this pio/sm is in use
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, /*rgbw=*/0);

    debouncer.debounce_gpio(PB_PIN);
    debouncer.set_debounce_time(PB_PIN, 10.0);    // 10ms debounce time
    debouncer.debounce_gpio(LIMIT_SW_PIN);
    debouncer.set_debounce_time(LIMIT_SW_PIN, 10.0);    // 10ms debounce time

    stepper_init(&stepper, L293_1A, L293_2A, L293_3A, L293_4A, /*steps/rev*/ 200, power);


    uint8_t state = PWRON;
    uint8_t next_state = PWRON;
    uint8_t entered = 1;
    uint8_t error = 0;
    uint8_t attempts, valid_checks;
    uint16_t step_pos = 0;

    int8_t pb_state = debouncer.read(PB_PIN);
    int8_t lim_sw_state = debouncer.read(LIMIT_SW_PIN);
    while (1) {

        pb_state = debouncer.read(PB_PIN);
        lim_sw_state = debouncer.read(LIMIT_SW_PIN);

        switch(state)
        {
            // Check that pushbutton and limit switch signals are valid, meaning the wires aren't broken
            case PWRON:
                if (entered)
                {
                    entered = 0;

                    // Color purple
                    put_pixel(urgb_u32(0xff, 0, 0xff));

                    attempts = 0;
                    valid_checks = 0;

                    sleep_ms(500);  // wait for things to stabilize

                }



                while (valid_checks < 5)
                {
                    pb_state = debouncer.read(PB_PIN);
                    lim_sw_state = debouncer.read(LIMIT_SW_PIN);                    

                    // If neither active high switch is on, they are valid
                    if (pb_state == 0 && lim_sw_state == 0)
                        valid_checks++;

                    // If too many attempts without success, enter error state
                    if (++attempts >= 10)
                    {
                        entered = 1;
                        state = ERROR;
                        error = PWRON_FAIL;
                        break;
                    }
                    sleep_ms(10);
                }

                if (valid_checks >= 5)
                {
                    // Perform homing and go to sleep after
                    entered = 1;
                    state = HOME_OUTER;
                    next_state = SLEEP;
                }                
                break;


            // Stop and signal that an error has occurred
            case ERROR:
                if (entered)
                {
                    entered = 0;

                    stepper_release(&stepper);  // Clear motor driver signals
                    gpio_put(L293_EN_PIN, 0);   // Disable motor driver

                    // Color red
                    put_pixel(urgb_u32(0x2f, 0, 0));
                }

                // If pushbutton pressed, flash out the error code

                if (pb_state)
                {
                    put_pixel(urgb_u32(0, 0, 0));
                    sleep_ms(500);

                    for (uint8_t n = error+1; n > 0; n--)
                    {
                        put_pixel(urgb_u32(0x2f, 0x09, 0x2f));
                        sleep_ms(250);
                        put_pixel(urgb_u32(0, 0, 0));
                        sleep_ms(250);                        
                    }

                    // If button is still held, exit error state
                    if (debouncer.read(PB_PIN))
                    {
                        put_pixel(urgb_u32(0x0F, 0x0F, 0x0F));
                        entered = 1;
                        state = SLEEP;

                        while (debouncer.read(PB_PIN) == 1);    // Wait until button is released to continue
                        break;
                    }

                    // Color red
                    put_pixel(urgb_u32(0x2f, 0, 0));                    
                }

                break;


            // Attempt to move the slider outward to find where the home is and park the slider in the default position
            case HOME_OUTER:
                if (entered)
                {
                    entered = 0;

                    gpio_put(L293_EN_PIN, 1);   // Enable motor driver

                    // Color blue
                    put_pixel(urgb_u32(0, 0, 0x0f));

                    // Attempt to move 300 steps, which is more than the full range of motion and should cause the end stop to trigger
                    if (step_limited(&stepper, 300, backward, STEPPER_DELAY_US) == 0)
                    {
                        // Homing failed, go to error state
                        entered = 1;
                        state = ERROR;
                        error = HOMING_FAIL;
                        break;
                    }

                    // Endstop was found. Move back a short distance and try homing again
                    stepper.step_delay_us = STEPPER_DELAY_US;
                    stepper_rotate_steps(&stepper, 8);

                    sleep_ms(50);   // wait long enough for limit switch state to update
                    if (step_limited(&stepper, 50, backward, STEPPER_DELAY_US) == 0)
                    {
                        // Homing failed, go to error state
                        entered = 1;
                        state = ERROR;
                        error = HOMING_FAIL;
                        break;                        
                    }

                    // Move back to home position
                    stepper_rotate_steps(&stepper, 8);
                    step_pos = 0;

                    stepper_release(&stepper);
                    gpio_put(L293_EN_PIN, 0);   // Disable motor driver

                    sleep_ms(50);   // wait long enough for limit switch state to update

                    // Enter whichever next state is specified
                    entered = 1;
                    state = next_state;
                    
                }

                break;

            // Idle in low power state until a feeding cycle is requested by alarm or user pushing button

            /* TODO
                    Enter sleep mode, wake up if alarm triggered or button pressed 
            */
            case SLEEP:
                if (entered)
                {
                    entered = 0;            
                    gpio_put(L293_EN_PIN, 0);   // Disable motor driver

                    // dim LED during sleep
                    put_pixel(urgb_u32(1, 1, 1));
                }

                // Button pressed
                if (debouncer.read(PB_PIN) == 1)
                {
                    put_pixel(urgb_u32(0x10, 0x10, 0x10));
                    sleep_ms(500);

                    // If button is still held, it's a long press
                    if (debouncer.read(PB_PIN) == 1)
                    {
                        // Do alarm setup
                        // abort if we're at the limit of alarms already
                        if (alarm_count >= MAX_ALARMS)
                            break;

                        if (!rtc_get_datetime(&current_time))
                        {
                            // RTC is not running
                            put_pixel(urgb_u32(0xff, 0x00, 0x00)); 
                            while(1);
                        }                       

                        feed_alarms[alarm_count] = current_time;
                        alarm_count++;

                        // Acknowledge that an alarm was set. Green light
                        put_pixel(urgb_u32(0x00, 0x50, 0x00));   
                        sleep_ms(2000);                     

                        // Wait for button to be released before continuing
                        while (debouncer.read(PB_PIN) == 1);
                        put_pixel(urgb_u32(0x00, 0x00, 0x00)); 

                        break;
                    }
                    else
                    {   
                        // Short press, perform homing and then dispense
                        entered = 1;
                        state = HOME_OUTER;
                        next_state = DISPENSE;
                        break;
                    }
                }



                rtc_get_datetime(&current_time);

                // Check if an alarm is set and matching the current time
                for (uint8_t c = 0; c < alarm_count; c++)
                {
                    if (is_same_time_hms(&current_time, &feed_alarms[c]))
                    {
                        // Time matched. Wait one second so that time won't match anymore
                        sleep_ms(1000);

                        // Start feed cycle
                        entered = 1;
                        state = HOME_OUTER;
                        next_state = DISPENSE;
                        break;                        
                    }
                }

                break;

            case DISPENSE:
                if (entered)
                {
                    entered = 0;            
                    gpio_put(L293_EN_PIN, 1);   //Enable motor driver

                    // color yellow
                    put_pixel(urgb_u32(0x1f, 0x1f, 0));

                    // Move from home position to loading spot
                    if (step_limited(&stepper, 102, forward, STEPPER_DELAY_US) != 0)
                    {
                        // Should not have hit an end stop here
                        entered = 1;
                        state = ERROR;      
                        error = END_UNEXPECTED;    
                        break;                  
                    }
                    step_pos = 102;

                    // do a little jiggle to help food fall in
                    stepper.step_delay_us = STEPPER_DELAY_US;
                    for (uint8_t n = 0; n < 10; n++)
                    {
                        stepper_rotate_steps(&stepper, 5);
                        sleep_ms(10);
                        stepper_rotate_steps(&stepper, -5);
                        sleep_ms(10);
                    }

                    stepper_release(&stepper);  // Don't hold position
                    sleep_ms(1000);  // Wait for food to be loaded  

                    // Move from loading spot halfway home
                    // Move at half speed at first, this part is where highest motor torque is needed
                    if (step_limited(&stepper, step_pos/2, backward, STEPPER_DELAY_US/2) != 0)
                    {
                        // Should not have hit an end stop here
                        entered = 1;
                        state = ERROR;      
                        error = END_UNEXPECTED;     
                        break;                 
                    }      

                    // // Normal speed the rest of the way to home                   
                    // if (step_limited(&stepper, step_pos/2, backward, STEPPER_DELAY_US) != 0)
                    // {
                    //     // Should not have hit an end stop here
                    //     entered = 1;
                    //     state = ERROR;      
                    //     error = END_UNEXPECTED;     
                    //     break;                 
                    // }           


                    // Perform homing to complete feeding cycle and confirm slider is not stuck
                    entered = 1;
                    state = HOME_OUTER;
                    next_state = SLEEP;         

                }       
                break;         


            case TEST:
                // Display button status with LED colors
                put_pixel(urgb_u32(0x1f*pb_state, 0x1f*lim_sw_state, 0));

                break;
            

            default:
                state = PWRON;
        }

        sleep_ms(10);
    }
}
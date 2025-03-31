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

// Time in microseconds to wait between motor steps. Lower time value = faster motor movement.
// Motor starts at maximum delay (slowest movement)
#define STEPPER_MIN_DELAY_US 1000   
#define STEPPER_MAX_DELAY_US 2500
#define SPEED_LEVELS 32             // How finely speed is incremented between minimum and maximum
#define MAX_ALARMS 12               // Maximum number of alarms that can be set concurrently

#define SPEED_INC ((STEPPER_MAX_DELAY_US - STEPPER_MIN_DELAY_US) / SPEED_LEVELS)

Debounce debouncer;
stepper_t stepper;

datetime_t current_time;
datetime_t feed_alarms[MAX_ALARMS];
uint8_t alarm_count = 0;

int16_t stepper_pos;
int8_t stepper_speed;
int8_t stepper_acceleration;
uint16_t step_delay = STEPPER_MAX_DELAY_US;

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

// Apply acceleration and speed limit to the current speed value
void update_speed(void)
{
    // If speed will be increased
    if (stepper_acceleration >= 0)
    {
        // Make sure speed doesn't exceed limit
        if (stepper_speed < SPEED_LEVELS-1)
            stepper_speed += stepper_acceleration;
    }
    else
    // Negative acceleration, slowing down
    {
        stepper_speed += stepper_acceleration;

        if (stepper_speed < 0)
            stepper_speed = 0;
    }

    step_delay = STEPPER_MAX_DELAY_US - (stepper_speed * SPEED_INC);
}

// Step the specified number of times, unless the end stops are triggered
uint8_t stepper_move(stepper_t *s, uint16_t target_steps, stepper_direction_t direction, uint8_t ignore_endstops)
{
    gpio_put(L293_EN_PIN, 1);   // Make sure motor driver is enabled

    while (true)
    {
        // Return with error code if end stop is triggered and this behaviour is enabled
        if (!ignore_endstops && debouncer.read(LIMIT_SW_PIN) == 1)
            return 1;
            
        stepper_step_once(s, direction);
        target_steps--;

        if (target_steps == 0)
        {
            // Delay to hold position temporarily
            sleep_us(STEPPER_MAX_DELAY_US);
            stepper_release(s); 
            return 0;
        }

        // If we are near the end of travel and the speed is over half, start decelerating
        if (target_steps < SPEED_LEVELS/2 && stepper_speed >= SPEED_LEVELS/2)
            stepper_acceleration = -8;

        update_speed();
        sleep_us(step_delay);
    }
}

// Attempt to find outermost home and park after
uint8_t home_outer(void)
{
    stepper_acceleration = 1;
    stepper_speed = 0;

    // Attempts to move 300 steps, which is more than the full range of motion and should cause the end stop to trigger

    if (stepper_move(&stepper, 300, backward, 0) == 0)
    {
        return 1;
    }

    // Endstop was found. Move back to home position
    stepper_speed = 0;
    stepper_move(&stepper, 8, forward, 1);    
    sleep_ms(50);   // wait long enough for limit switch state to update  

    stepper_speed = 0;        
    if (stepper_move(&stepper, 300, backward, 0) == 0)
    {
        return 1;
    }
    else
    {
        // Endstop was found. Move back to home position
        stepper_speed = 0;
        stepper_move(&stepper, 8, forward, 1);    

        stepper_pos = 0;    // We're parked at home, so position is 0

        stepper_release(&stepper);
        gpio_put(L293_EN_PIN, 0);   // Disable motor driver

        sleep_ms(50);   // wait long enough for limit switch state to update        
        return 0;
    }
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
    ERROR,
    TEST
};

// Error codes
enum
{
    NONE,
    PWRON_FAIL,
    END_UNEXPECTED,
    HOMING_FAIL,
    JAMMED
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
    uint8_t entered = 1;
    uint8_t error = 0;
    uint8_t attempts, valid_checks;

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
                    // Home and go to sleep after
                    if (home_outer() == 1)
                    {
                        // Homing failed, go to error state
                        entered = 1;
                        state = ERROR;
                        error = HOMING_FAIL;
                        break;        
                    }                     


                    entered = 1;
                    state = SLEEP;
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
                        state = DISPENSE;
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
                        state = DISPENSE;
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
                    
                    // home
                    if (home_outer() == 1)
                    {
                        // Homing failed, go to error state
                        entered = 1;
                        state = ERROR;
                        error = HOMING_FAIL;
                        break;        
                    }  

                    // Move from home position to loading spot
                    stepper_speed = 0;
                    stepper_acceleration = 1;
                    if (stepper_move(&stepper, 102, forward, 0) != 0)
                    {
                        // Should not have hit an end stop here
                        entered = 1;
                        state = ERROR;      
                        error = END_UNEXPECTED;    
                        break;                  
                    }

                    // do a little jiggle to help food fall in
                    // *** this jiggle produces the most noise. It may not be necessary
                    // stepper_acceleration = 4;                    
                    // for (uint8_t n = 0; n < 10; n++)
                    // {
                    //     stepper_speed = 0;
                    //     stepper_move(&stepper, 5, forward, 0);                        
                    //     sleep_ms(10);

                    //     stepper_speed = 0;
                    //     stepper_move(&stepper, 5, backward, 0);                        
                    //     sleep_ms(10);
                    // }

                    stepper_release(&stepper);  // Don't hold position
                    sleep_ms(1000);  // Wait for food to be loaded  


                    entered = 1;    // we will change states after

                    // Perform homing to complete feeding cycle and confirm slider is not stuck
                    // Home and go to sleep after
                    if (home_outer() == 1)
                    {
                        // Slider is jammed. Try to move it back to the inner endstop
                        stepper_speed = 0;
                        stepper_acceleration = 1;
                        if (stepper_move(&stepper, 300, forward, 0) == 0)
                        {
                            // Homing failed, go to error state
                            state = ERROR;
                            error = HOMING_FAIL;
                            break;  
                        }                        

                        // Back away from the endstop
                        stepper_speed = 0;
                        stepper_move(&stepper, 20, backward, 1);
                        sleep_ms(50);   // wait long enough for limit switch state to update  

                        // Try to home once more, to finish the feed cycle
                        if(home_outer() == 1)
                        {
                            // Homing failed, go to error state
                            state = ERROR;
                            error = JAMMED;
                            break;  
                        }
      
                    }      

                    entered = 1;
                    state = SLEEP;         

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
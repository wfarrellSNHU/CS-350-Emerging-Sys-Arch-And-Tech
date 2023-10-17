/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONT=-/IBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/Timer.h>

// States
// GP_Start is the starting of the SM
// GP_Wait is the wait state
// GP_SOS is the SOS state to display the SOS Morse code
// GP_OK is the OK state to display the OK Morse code after the button press
enum GP_States { GP_Start, GP_Wait, GP_SOS, GP_OK } GP_State; // Enum holding the state of the SM

unsigned int LetterWorker = 0x00;
// SOS Letters
// 0x01 = S1 character
// 0x02 = O character
// 0x04 = S2 character

// OK Letters
// 0x01 = O character
// 0x02 = K character

// Wait Mode
// 0x01 = 500ms wait
// 0x02 = 500ms wait
// 0x04 = 500ms wait
// 0x08 = 500ms wait
// 0x100 = 500ms wait
// 0x200 = 500ms wait
// 0x400 = 500ms wait

unsigned int BitWorker = 0x00; // keeps track of which part of the character or pause between
// S1 Character
// 0x01 = 500ms dot
// 0x02 = 500ms wait
// 0x04 = 500ms dot
// 0x08 = 500ms wait
// 0x10 = 500ms dot
// 0x20 = 500ms wait
// 0x40 = 500ms wait
// 0x80 = 500ms wait

// S2 Character
// 0x01 = 500ms dot
// 0x02 = 500ms wait
// 0x04 = 500ms dot
// 0x08 = 500ms wait
// 0x10 = 500ms dot

// O Character
// 0x01 = 500ms dash
// 0x02 = 500ms dash
// 0x04 = 500ms dash
// 0x08 = 500ms wait
// 0x10 = 500ms dash
// 0x20 = 500ms dash
// 0x40 = 500ms dash
// 0x80 = 500ms wait
// 0x100 = 500ms dash
// 0x200 = 500ms dash
// 0x400 = 500ms dash
// 0x800 = 500ms wait
// 0x1000 = 500ms wait
// 0x2000 = 500ms wait

// K Character
// 0x01 = 500ms dash
// 0x02 = 500ms dash
// 0x04 = 500ms dash
// 0x08 = 500ms wait
// 0x10 = 500ms dot
// 0x20 = 500ms wait
// 0x40 = 500ms dash
// 0x80 = 500ms dash
// 0x100 = 500ms dash

unsigned char StateToggler = 0x00; // Start in SOS Mode, 0/x01 is OK Mode

void TickGP_Toggle()
{
    printf("GP_State = %d LetterWorker = %x BitWorker = %x \n", GP_State, LetterWorker, BitWorker);
    switch(GP_State) { // Transitions
    case GP_Start:
        GP_State = GP_SOS;
        LetterWorker = 0x01;
        BitWorker = 0x01;
        break;

    case GP_OK:
        // If we are done, go to GP_Wait
        if (LetterWorker == 0x04) {
            printf("GP_Wait and switch!\n");
            GP_State = GP_Wait;
            LetterWorker = 0x01;
            BitWorker = 0x01;
        }
        break;

    case GP_SOS:
        // If we are done, go to GP_Wait
        if (LetterWorker == 0x08) {
            printf("GP_Wait switch!\n");
            GP_State = GP_Wait;
            LetterWorker = 0x01;
            BitWorker = 0x01;
        }
        break;

    case GP_Wait:
        // If we are done waiting 7 times
        if (LetterWorker == 0x800 ) {
            // If we are in SOS mode
            if (StateToggler == 0x00) {
                printf("GP_SOS switch!\n");
                GP_State = GP_SOS;
                LetterWorker = 0x01;
                BitWorker = 0x01;

            }
            // Else we're in OK mode
            else {
                printf("GP_OK switch!\n");
                GP_State = GP_OK;
                LetterWorker = 0x01;
                BitWorker = 0x01;
            }
        }
        break;
    }

    switch(GP_State) { // Actions
    case GP_Start:
        break;

    case GP_SOS:
        switch(LetterWorker) {
        case 0x01: // S1
            switch (BitWorker) {
            case 0x01: // dot
                // Turn on Red
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                BitWorker = BitWorker << 1;
                break;

            case 0x02: // wait
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                BitWorker = BitWorker << 1;
                break;

            case 0x04: // dot
                // Turn on Red
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                BitWorker = BitWorker << 1;
                break;

            case 0x08: // wait
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                BitWorker = BitWorker << 1;
                break;

            case 0x10: // dot
                // Turn on Red
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                BitWorker = BitWorker << 1;
                break;

            case 0x20: // wait
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                BitWorker = BitWorker << 1;
                break;

            case 0x40: // wait
                BitWorker = BitWorker << 1;
                break;

            case 0x80: // wait
                BitWorker = 0x01;
                LetterWorker = LetterWorker << 1;
                break;
            }
            break;

            case 0x02: // O
                switch (BitWorker) {
                case 0x01: // dash
                    // Turn on Green
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x02: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x04: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x08: // wait
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x10: // Dash
                    // Turn on Green
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x20: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x40: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x80: // Wait
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x100: // Dash
                    // Turn on Green
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x200: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x400: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x800: // Wait
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x1000: // Wait
                    BitWorker = BitWorker << 1;
                    break;

                case 0x2000: // Wait
                    BitWorker = 0x01;
                    LetterWorker = LetterWorker << 1;
                    break;

                }
                break;

                case 0x04: // S2
                    switch (BitWorker) {
                    case 0x01: // dot
                        // Turn on Red
                        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x02: // wait
                        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x04: // dot
                        // Turn on Red
                        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x08: // wait
                        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x10: // dot
                        // Turn on Red
                        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                        BitWorker = 0x01;
                        LetterWorker = LetterWorker << 1;
                        break;

                    }
                    break;

        }
        break;

        case GP_OK:
            switch(LetterWorker) {
            case 0x01: // O
                switch (BitWorker) {
                case 0x01: // dash
                    // Turn on Green
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x02: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x04: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x08: // wait
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x10: // Dash
                    // Turn on Green
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x20: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x40: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x80: // Wait
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x100: // Dash
                    // Turn on Green
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x200: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x400: // Dash
                    BitWorker = BitWorker << 1;
                    break;

                case 0x800: // Wait
                    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                    BitWorker = BitWorker << 1;
                    break;

                case 0x1000: // Wait
                    BitWorker = BitWorker << 1;
                    break;

                case 0x2000: // Wait
                    BitWorker = 0x01;
                    LetterWorker = LetterWorker << 1;
                    break;

                }
                break;

                case 0x02: // K
                    switch (BitWorker) {
                    case 0x01: // dash
                        // Turn on Green
                        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x02: // dash
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x04: // dash
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x08: // wait
                        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x10: // dot
                        // Turn on Red
                        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x20: // wait
                        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x40: // dash
                        // Turn on Green
                        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x80: // dash
                        BitWorker = BitWorker << 1;
                        break;

                    case 0x100: // dash
                        BitWorker = 0x01;
                        LetterWorker = LetterWorker << 1;
                        break;
                    }
                    break;

            }
            break;

            case GP_Wait:
                // Turn everything off
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
                LetterWorker = LetterWorker << 1;
                break;
    }

}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    // GPIO_toggle(CONFIG_GPIO_LED_0);



}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
    // GPIO_toggle(CONFIG_GPIO_LED_1);

    printf("Time to toggle!\n");
    if (StateToggler == 0x00)
        StateToggler = 0x01;
    else
        StateToggler = 0x00;
}

/*
 *  ======== timerCallback ========
 *  Callback function for the timer interrupt on CONFIG_TIMER_0.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TickGP_Toggle();
    printf("Tick...");
}

/*
 *  ======== initTimer ========
 *  Initialize the timer on CONFIG_TIMER_0.
 */
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initTimer();
    return (NULL);
}

/******************************************************************************
* File Name:   main.c
*
* Description: This code captures PDM/PCM audio data from microphone and 
*              sends it via UART to PC for WAV file recording
*              Sample rate: 16kHz, Mono, 16-bit
*
* Related Document: See README.md 
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "stdlib.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Define how many samples in a frame */
#define FRAME_SIZE                  (512)   // Reduced for 16kHz
/* Desired sample rate: 16kHz for audio streaming */
#define SAMPLE_RATE_HZ              16000u
/* Decimation Rate of the PDM/PCM block. Typical value is 64 */
#define DECIMATION_RATE             64u
/* Audio Subsystem Clock for 16kHz: 24.576 MHz */
#define AUDIO_SYS_CLOCK_HZ          24576000u
/* PDM/PCM Pins */
#define PDM_DATA                    P10_5
#define PDM_CLK                     P10_4

/* UART Configuration for high-speed audio data transmission */
#define UART_BAUD_RATE              460800u  // High baud rate for audio data

/* Audio streaming control */
#define AUDIO_START_MARKER          0xAA55
#define AUDIO_END_MARKER            0x55AA

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void button_isr_handler(void *arg, cyhal_gpio_event_t event);
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event);
void clock_init(void);
void handle_error(void);
void send_audio_data_uart(int16_t *audio_data, uint32_t size);
void send_wav_header(void);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Interrupt flags */
volatile bool button_flag = false;
volatile bool pdm_pcm_flag = false;
volatile bool audio_streaming = false;

/* Volume variables */

/* HAL Objects */
cyhal_pdm_pcm_t pdm_pcm;
cyhal_clock_t   audio_clock;
cyhal_clock_t   pll_clock;

/* HAL Config - MONO configuration for single channel */
const cyhal_pdm_pcm_cfg_t pdm_pcm_cfg = 
{
    .sample_rate     = SAMPLE_RATE_HZ,
    .decimation_rate = DECIMATION_RATE,
    .mode            = CYHAL_PDM_PCM_MODE_LEFT,  
    .word_length     = 16,  /* bits */
    .left_gain       = 10,  
    .right_gain      = 0,   
};

/* Button callback structure */
cyhal_gpio_callback_data_t cb_data =
{
    .callback = button_isr_handler,
    .callback_arg = NULL
};

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts */
    __disable_irq();
    CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: send_audio_data_uart
********************************************************************************
* Summary:
* Sends audio data via UART to PC
*
* Parameters:
*  audio_data: pointer to audio data buffer
*  size: number of samples to send
*
* Return:
*  void
*
*******************************************************************************/
void send_audio_data_uart(int16_t *audio_data, uint32_t size)
{
    // Send start marker
    uint8_t start_marker_bytes[2] = {(AUDIO_START_MARKER & 0xFF), (AUDIO_START_MARKER >> 8)};
    for (int i = 0; i < 2; i++)
    {
        if (CY_RSLT_SUCCESS != cyhal_uart_putc(&cy_retarget_io_uart_obj, start_marker_bytes[i]))
        {
            return; 
        }
    }
    
    // Send audio data (16-bit samples as bytes)
    for (uint32_t i = 0; i < size; i++)
    {
        // Send (little endian)
        if (CY_RSLT_SUCCESS != cyhal_uart_putc(&cy_retarget_io_uart_obj, 
                                               (uint8_t)(audio_data[i] & 0xFF)))
        {
            return;
        }
        
        
        if (CY_RSLT_SUCCESS != cyhal_uart_putc(&cy_retarget_io_uart_obj, 
                                               (uint8_t)(audio_data[i] >> 8)))
        {
            return;
        }
    }
    
    // Send end marker
    uint8_t end_marker_bytes[2] = {(AUDIO_END_MARKER & 0xFF), (AUDIO_END_MARKER >> 8)};
    for (int i = 0; i < 2; i++)
    {
        if (CY_RSLT_SUCCESS != cyhal_uart_putc(&cy_retarget_io_uart_obj, end_marker_bytes[i]))
        {
            return;
        }
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* Main function that initializes the system and handles audio streaming
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    int16_t audio_frame[FRAME_SIZE] = {0};

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Init the clocks */
    clock_init();

    /* Initialize retarget-io with flow control for high-speed UART */
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, 
                                    CYBSP_DEBUG_UART_RX,
                                    CYBSP_DEBUG_UART_CTS,
                                    CYBSP_DEBUG_UART_RTS,
                                    UART_BAUD_RATE);

    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }


    /* Initialize the User Button */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, 
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, 
                           CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
    /* Initialize the PDM/PCM block */
    result = cyhal_pdm_pcm_init(&pdm_pcm, PDM_DATA, PDM_CLK, &audio_clock, &pdm_pcm_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    
    cyhal_pdm_pcm_register_callback(&pdm_pcm, pdm_pcm_isr_handler, NULL);
    cyhal_pdm_pcm_enable_event(&pdm_pcm, CYHAL_PDM_PCM_ASYNC_COMPLETE, 
                              CYHAL_ISR_PRIORITY_DEFAULT, true);
    cyhal_pdm_pcm_start(&pdm_pcm);
    
    /* Clear screen and show initial message */
    printf("\x1b[2J\x1b[;H");
    printf("************************************************\r\n");
    printf("PDM/PCM Audio Streaming to PC via UART\r\n");
    printf("Sample Rate: 16kHz, Mono, 16-bit\r\n");
    printf("Baud Rate: %lu\r\n", (unsigned long)UART_BAUD_RATE);
    printf("************************************************\r\n");
    printf("Press User Button to start/stop audio streaming\r\n\n");

    /* Start first read */
    cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
    
    for(;;)
    {
       
        if (pdm_pcm_flag)
        {
       
            pdm_pcm_flag = false;
        
            if (audio_streaming)
            {
                send_audio_data_uart(audio_frame, FRAME_SIZE);
            }

        
            cyhal_pdm_pcm_read_async(&pdm_pcm, audio_frame, FRAME_SIZE);
        }

        /* Handle button press to start/stop streaming */
        if (button_flag)
        {
            /* Reset button flag */
            button_flag = false;

           
            audio_streaming = !audio_streaming;
            
            if (audio_streaming)
            {
                printf("Audio streaming STARTED\r\n");
            }
            else
            {
                printf("Audio streaming STOPPED\r\n");
            }
        }

        /* Enter sleep mode to save power */
        cyhal_syspm_sleep();
    }
}

/*******************************************************************************
* Function Name: button_isr_handler
********************************************************************************
* Summary:
* Button ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void button_isr_handler(void *arg, cyhal_gpio_event_t event)
{
    (void) arg;
    (void) event;
    button_flag = true;
}

/*******************************************************************************
* Function Name: pdm_pcm_isr_handler
********************************************************************************
* Summary:
* PDM/PCM ISR handler. Set a flag to be processed in the main loop.
*
* Parameters:
*  arg: not used
*  event: event that occurred
*
*******************************************************************************/
void pdm_pcm_isr_handler(void *arg, cyhal_pdm_pcm_event_t event)
{
    (void) arg;
    (void) event;
    pdm_pcm_flag = true;
}
/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
* Initialize the clocks in the system.
*
*******************************************************************************/
void clock_init(void)
{
    /* Initialize the PLL */
    cyhal_clock_reserve(&pll_clock, &CYHAL_CLOCK_PLL[0]);
    cyhal_clock_set_frequency(&pll_clock, AUDIO_SYS_CLOCK_HZ, NULL);
    cyhal_clock_set_enabled(&pll_clock, true, true);

    /* Initialize the audio subsystem clock (CLK_HF[1]) 
     * The CLK_HF[1] is the root clock for the I2S and PDM/PCM blocks */
    cyhal_clock_reserve(&audio_clock, &CYHAL_CLOCK_HF[1]);

    /* Source the audio subsystem clock from PLL */
    cyhal_clock_set_source(&audio_clock, &pll_clock);
    cyhal_clock_set_enabled(&audio_clock, true, true);
}

/* [] END OF FILE */
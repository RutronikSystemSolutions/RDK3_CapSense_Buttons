/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK3_CapSense_Buttons
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-08-18
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_capsense.h"

#define CAPSENSE_INTR_PRIORITY      (7u)
#define EZI2C_INTR_PRIORITY         (6u)

void handle_error(void);
static uint32_t initialize_capsense(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);
static void capsense_callback();
static void process_touch(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;
volatile bool capsense_scan_complete = false;


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize the User LEDs */
    result = cyhal_gpio_init(LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(LED3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_OFF);

    /*CapSense Initializations*/
    initialize_capsense_tuner();
    result = initialize_capsense();
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RDK3 CapSense CSX Buttons Example.\r\n");
    printf("Touch the CSB1, CSB2, CSB3 sensors on the board.\r\n");

    /* Initiate first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for (;;)
    {
    	/*Check if the CSD slider scan is complete*/
        if (capsense_scan_complete)
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Initiate next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

            capsense_scan_complete = false;
        }
    }
}

/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  Initializes interface between Tuner GUI and PSoC 6 MCU.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_rslt_t result;

    /* Configure Capsense Tuner as EzI2C Slave */
    sEzI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    sEzI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.slave_address = 8U;

    sEzI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    sEzI2C_cfg.enable_wake_from_sleep = false;
    sEzI2C_cfg.slave1_cfg = sEzI2C_sub_cfg;
    sEzI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    sEzI2C_cfg.two_addresses = false;

    result = cyhal_ezi2c_init(&sEzI2C, ARDU_SDA, ARDU_SCL, NULL, &sEzI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t initialize_capsense(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
        {
            .intrSrc = CAPSENSE_IRQ,
            .intrPriority = CAPSENSE_INTR_PRIORITY,
        };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
    NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
    NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E, capsense_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CAPSENSE_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_callback()
********************************************************************************
* Summary:
*  This function sets a flag to indicate end of a CapSense scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t* : pointer to active sensor details.
*
*******************************************************************************/
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    capsense_scan_complete = true;
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    uint32_t button0_status;
    uint32_t button1_status;
    uint32_t button2_status;
    static _Bool csb1_status = false;
    static _Bool csb2_status = false;
    static _Bool csb3_status = false;

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON0_WDGT_ID,CY_CAPSENSE_BUTTON0_SNS0_ID,&cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON1_WDGT_ID,CY_CAPSENSE_BUTTON1_SNS0_ID,&cy_capsense_context);

    /* Get button 2 status */
    button2_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON2_WDGT_ID,CY_CAPSENSE_BUTTON2_SNS0_ID,&cy_capsense_context);

    /* Detect new touch on Button0 */
	if(button0_status)
	{
		cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_ON);
		if(!csb1_status)
		{
			printf("CSB1 activated.\r\n");
			csb1_status = true;
		}
	}
	else
	{
		cyhal_gpio_write((cyhal_gpio_t)LED1, CYBSP_LED_STATE_OFF);
		if(csb1_status)
		{
			printf("CSB1 deactivated.\r\n");
			csb1_status = false;
		}
	}

    /* Detect new touch on Button1 */
    if (button1_status)
	{
		cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_ON);
		if(!csb2_status)
		{
			printf("CSB2 activated.\r\n");
			csb2_status = true;
		}
	}
	else
	{
		cyhal_gpio_write((cyhal_gpio_t)LED2, CYBSP_LED_STATE_OFF);
		if(csb2_status)
		{
			printf("CSB2 deactivated.\r\n");
			csb2_status = false;
		}
	}

    /* Detect new touch on Button2 */
    if (button2_status)
	{
		cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_ON);
		if(!csb3_status)
		{
			printf("CSB3 activated.\r\n");
			csb3_status = true;
		}
	}
	else
	{
		cyhal_gpio_write((cyhal_gpio_t)LED3, CYBSP_LED_STATE_OFF);
		if(csb3_status)
		{
			printf("CSB3 deactivated.\r\n");
			csb3_status = false;
		}
	}

}

void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */

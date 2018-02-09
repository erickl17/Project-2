/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

// Function dorOrDash returns 0 if time is dot length and returns 1 if time is dash length
int dotOrDash(int time)
{
	if (time < 15630)
	{
		return 0;
	}
	else if (time > 31300 & time < 46900){
		return 1;
	}
}

int shortMedLong(int time)
{
	if (time < 15630)
	{
		return 0;
	}
	else if (time > 31300 & time < 46900){
		return 1;
	}
	else if (time > 6400)
	{
		return 2;
	}
}

int main(void)
{
	unsigned short cal_value = 0;
	unsigned char light_value = 0;

	hardware_init();

	// Setup Timer
	SIM_SCGC6 |= (1<<26);	// Clock Enable TPM2
	SIM_SOPT2 |= (0x2 << 24);	// Set TPMSRC to OSCERCLK

	TPM2_C0SC |= (0x1 << 2) | (0x1 << 4);
	TPM2_MOD = 62499;
	TPM2_C0V = 62499;
	TPM2_SC |= 0x7; // Prescaler

	TPM2_C1SC |= (0x1 << 2) | (0x1 << 4);
	TPM2_C1V = 62499;

	//PORTE_PCR22 |= 0x300;	// Set MUX to TPM2_CH0

	// Setup Channel
	//TPM2_C0SC |= (0x1 << 2);	// Toggle Input Capture

	//TPM2_SC |= 0x01 << 3 | 0x7; // Start the clock!


	// Setup Analog Input - Default is analog (PTE22), No need to do anything.


	SIM_SCGC5 |= (1<<12);
	// Setup LED Pin for GPIO
	PORTD_PCR5 &= ~0x700; // Clear First
	PORTD_PCR5 |= 0x700 & (1 << 8); // Set MUX bits

	// Setup Pin 5 as output
	GPIOD_PDDR |= (1 << 5);

    // Clock Gating
	SIM_SCGC5 |= (1<<13); // Enable Light Sensor I/O Port
	SIM_SCGC6 |= (1 << 27);	// Enables ADC module



	// Setup ADC Clock
	ADC0_CFG1 = 0;	// Default everything

	// ADC Calibration
	ADC0_SC3 = 0x07;	// Sets hardware averaging to max
	ADC0_SC3 = 0x80;	// Initiates calibration

	// Wait for Calibration to Complete (either COCO or CALF)
	while(!(ADC0_SC1A & 0x80)){	}

	// Write plus-side gain calibration registers
	cal_value = ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP3 + ADC0_CLP4 + ADC0_CLPS;
	cal_value = cal_value >> 1;	// Divided cal_value by 2
	cal_value |= 0x8000;	// Set MSB of cal_value
	ADC0_PG = cal_value;	// Sets plus-side gain PG calibration register to cal_value

	// Write minus-side gain calibration registers
	cal_value = 0;
	cal_value = ADC0_CLM0 + ADC0_CLM1 + ADC0_CLM2 + ADC0_CLM3 + ADC0_CLM4 + ADC0_CLMS;
	cal_value = cal_value >> 1;	// Divided cal_value by 2
	cal_value |= 0x8000;	// Set MSB of cal_value
	ADC0_MG = cal_value;	// Sets plus-side gain PG calibration register to cal_value

	ADC0_SC3 = 0; // Turn off hardware averaging

    /* This for loop should be replaced. By default this loop allows a single stepping. */
	unsigned int x = 0;
	unsigned int y = 0;
	unsigned int z = 0;
	while(1) {


		/*ADC0_SC1A = 0x03; // Set Channel, starts conversion.
		while(!(ADC0_SC1A & 0x80)){	}

		light_value = ADC0_RA; // Resets COCO */
		if(light_value < 250) {
			GPIOD_PDOR |= (1<<5);
			/*if (!(TPM2_SC & 0x08))
			{
				TPM2_SC |= (1 << 3);
			} */
			TPM2_SC |= (1 << 3);
			while (light_value < 250)
			{
				ADC0_SC1A = 0x03; // Set Channel, starts conversion.
				while(!(ADC0_SC1A & 0x80)){	}
				light_value = ADC0_RA; // Resets COCO

			}
			TPM2_SC &= (0 << 3);
			x = TPM2_CNT;
			TPM2_CNT = 0x00;
			y = dotOrDash(x);
		}
		else {
			GPIOD_PDOR &= ~(1<<5);
			/*if ((TPM2_SC & 0x08))
			{
				TPM2_SC &= (0 << 3);
				x = TPM2_CNT;
				TPM2_CNT = 0x00;
				y = dotOrDash(x);
			}*/
			TPM2_SC |= (1 << 3);
			while (light_value < 250)
			{
				ADC0_SC1A = 0x03; // Set Channel, starts conversion.
				while(!(ADC0_SC1A & 0x80)){	}
				light_value = ADC0_RA; // Resets COCO

			}
			TPM2_SC &= (0 << 3);
			x = TPM2_CNT;
			TPM2_CNT = 0x00;
			z = shortMedLong(x);
		}
	}
    /* Never leave main */
    return 0;
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

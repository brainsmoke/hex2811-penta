/*  HexWS2801z - an OctoWS2811 fork to drive (at most) 15 WS2801 LED strips at once
    using a Teensy 3.x
    by Erik Bosman <erik@minemu.org> Zero copy version, similar to the Fadecandy
    implementation.

    Based on:
    OctoWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include <string.h>
#include "HexWS2801z.h"

#define LATCH_DELAY 1000

uint32_t HexWS2801z::bufsize;
int HexWS2801z::freq;
DMAChannel HexWS2801z::dma2;
DMAChannel HexWS2801z::dma3;
uint16_t HexWS2801z::clockmask;
int HexWS2801z::skipClear;

static volatile uint8_t update_in_progress = 0;
static uint32_t update_completed_at = 0;


HexWS2801z::HexWS2801z(uint32_t bufsize, uint16_t clockmask, int skipClear, int freq)
{
	this->bufsize = bufsize;
	this->clockmask = clockmask;
	this->skipClear = skipClear;
	this->freq = freq;
}

void HexWS2801z::initGPIO(void)
{
	GPIOC_PDOR = (clockmask ^ 0xff) & 0xff;
	GPIOD_PDOR = ( (clockmask>>8) ^ 0xff) & 0xff;
}

void HexWS2801z::begin(void)
{
	initGPIO();
	// configure the 16 output pins

	pinMode(15, OUTPUT);	// strip #1
	pinMode(22, OUTPUT);	// strip #2
	pinMode(23, OUTPUT);	// strip #3
	pinMode(9, OUTPUT);	// strip #4
	pinMode(10, OUTPUT);	// strip #5
	pinMode(13, OUTPUT);	// strip #6
	pinMode(11, OUTPUT);	// strip #7
	pinMode(12, OUTPUT);	// strip #8

	pinMode(2, OUTPUT);	// strip #9
	pinMode(14, OUTPUT);	// strip #10
	pinMode(7, OUTPUT);	// strip #11
	pinMode(8, OUTPUT);	// strip #12
	pinMode(6, OUTPUT);	// strip #13
	pinMode(20, OUTPUT);	// strip #14
	pinMode(21, OUTPUT);	// strip #15
	pinMode(5, OUTPUT);	// strip #16

	// create the two waveforms for WS2811 low and high bits
	analogWriteResolution(8);
	analogWriteFrequency(3, this->freq);
	analogWriteFrequency(4, this->freq);
	analogWrite(3, 128);
	analogWrite(4, 128);

	// pin 16 triggers DMA(port B) on rising edge (configure for pin 3's waveform)
	CORE_PIN16_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3)|PORT_PCR_PE;
	pinMode(3, INPUT_PULLUP); // pin 3 no longer needed

//	// pin 4 triggers DMA(port A) on both edges of high duty waveform
//	CORE_PIN4_CONFIG = PORT_PCR_IRQC(3)|PORT_PCR_MUX(3);
	// pin 4 triggers DMA(port A) on falling edge
	CORE_PIN4_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(3)|PORT_PCR_PE;

	// DMA channel #2 writes the pixel data at 20% of the cycle
//	dma2.TCD->SADDR = frameBuffer;
	dma2.TCD->SADDR = NULL;
	dma2.TCD->SOFF = 2;
	dma2.TCD->ATTR_SRC = DMA_TCD_ATTR_SIZE_16BIT;
	dma2.TCD->SLAST = -bufsize;

	/* Send data to both PORT C and D in the same minor loop (executed after the same trigger( */

	#define PORT_DELTA ( (uint32_t)&GPIOD_PDOR - (uint32_t)&GPIOC_PDOR )
    dma2.TCD->DADDR = &GPIOC_PDOR;
	dma2.TCD->DOFF = PORT_DELTA;
                         /* loop GPIOC_PDOR, GPIOD_PDOR and back */
	dma2.TCD->ATTR_DST = ((31 - __builtin_clz(PORT_DELTA*2)) << 3) | DMA_TCD_ATTR_SIZE_8BIT;
	dma2.TCD->DLASTSGA = 0;

	dma2.TCD->NBYTES = 2;
	dma2.TCD->BITER = bufsize / 2;
	dma2.TCD->CITER = bufsize / 2;

	dma2.disableOnCompletion();

	// DMA channel #3
	dma3.TCD->SADDR = &clockmask;
	dma3.TCD->SOFF = 0;
	dma3.TCD->ATTR_SRC = DMA_TCD_ATTR_SIZE_16BIT;
	dma3.TCD->SLAST = 0;

    dma3.TCD->DADDR = &GPIOC_PSOR;
	dma3.TCD->DOFF = PORT_DELTA;
                         /* loop GPIOC_PSOR, GPIOD_PSOR and back */
	dma3.TCD->ATTR_DST = ((31 - __builtin_clz(PORT_DELTA*2)) << 3) | DMA_TCD_ATTR_SIZE_8BIT;
	dma3.TCD->DLASTSGA = 0;

	dma3.TCD->NBYTES = 2;
	dma3.TCD->BITER = bufsize/2;
	dma3.TCD->CITER = bufsize/2;
	dma3.disableOnCompletion();
	dma3.interruptAtCompletion();

#ifdef __MK20DX256__
	MCM_CR = MCM_CR_SRAMLAP(1) | MCM_CR_SRAMUAP(0);
	AXBS_PRS0 = 0x1032;
#endif

	// route the edge detect interrupts to trigger the 3 channels
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTA);

	// enable a done interrupts when channel #3 completes
	dma3.attachInterrupt(isr);
	//pinMode(9, OUTPUT); // testing: oscilloscope trigger
}

void HexWS2801z::isr(void)
{
	dma3.clearInterrupt();
	initGPIO();
	update_completed_at = micros();
	update_in_progress = 0;
}

int HexWS2801z::busy(void)
{
	if (update_in_progress) return 1;
	// busy for 50 us after the done interrupt, for WS2811 reset
	if (micros() - update_completed_at < LATCH_DELAY) return 1;
	return 0;
}

void HexWS2801z::show(void *frameBuffer)
{
	while (update_in_progress) ; 
	update_in_progress = 1;

	if (!this->skipClear)
	{
		uint32_t i;
		uint16_t *b16 = (uint16_t *)frameBuffer;
		for (i=0; i<bufsize/2; i++)
			b16[i] &= ~clockmask;
	}

	dma2.TCD->SADDR = frameBuffer;

	uint32_t sc = FTM1_SC;
	FTM1_SC = sc & 0xE7;	// stop FTM1 timer

	FTM1_CNTIN = FTM1_MOD-1;
	FTM1_CNT = 0xbeef;      // FTM1_CNT == FTM1_MOD
	FTM1_CNTIN = 0;

	PORTB_ISFR = (1<<0);    // clear any prior rising edge
	PORTA_ISFR = (1<<13);	// clear any prior high duty falling edge
	dma2.enable();		// enable all 3 DMA channels
	dma3.enable();
	// wait for WS2801 reset
	while (micros() - update_completed_at < LATCH_DELAY) ;
	FTM1_SC = sc;		// restart FTM1 timer
}


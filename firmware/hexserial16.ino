/*
 * Drive 16 UART LED strip drivers chips at once
 *
 * Protocol (both in and out): ( [16 bit brightness]* [ FF FF FF F0 ] )*
 *
 * Brightness must be little endian integers in the (inclusive) range [0 .. 0xFF00]
 *
 * [ FF FF FF F0 ] is an end of frame marker and allows the protocol to synchronize
 * in the event of an uneven number of bytes being written to the serial port
 *
 * The firmware is completely agnostic about the color ordering
 * Each frame is divided into 16 strips each of N_LEDS_PER_STRIP*LED_VALUES values.
 * Sending less than 16 strips worth of data will leave the old values in place.
 *
 * To reduce flickering, it is recommended not to send brightness values in the range [1 .. 0x1f]
 *
 */

#include <HexSerialz.h>
#include <usb_dev.h>

#define LED_VALUES 4
#define N_BYTES_PER_VALUE 2
#define N_LEDS_PER_STRIP 75

#define N_DATA_BYTES_PER_STRIP (N_BYTES_PER_VALUE * N_LEDS_PER_STRIP * LED_VALUES)
#define N_BYTES_PER_STRIP (N_DATA_BYTES_PER_STRIP + 4)
#define N_DATA_BAUDS_PER_STRIP (10 * N_DATA_BYTES_PER_STRIP)
#define N_BAUDS_PER_STRIP (10 * N_BYTES_PER_STRIP)
#define N_STRIPS 16

uint16_t buf_a[N_BAUDS_PER_STRIP];
uint16_t buf_b[N_BAUDS_PER_STRIP];
uint16_t *cur, *next;
HexSerialz *hex;

void init_buf(uint16_t *buf)
{
	uint32_t i;

	for (i=N_BAUDS_PER_STRIP-40; i<N_BAUDS_PER_STRIP; i++)
		buf[i] = 0xffff;

	for (i=N_BAUDS_PER_STRIP-9; i<N_BAUDS_PER_STRIP-5; i++)
		buf[i] = 0x0000;

	for (i=0; i<N_BAUDS_PER_STRIP; i+=10)
	{
		buf[i] = 0x0000;
		buf[i+9] = 0xffff;
	}
}

static usb_packet_t *rx_packet=NULL;
static int rx_i=0, fastpath=0;

static int usb_getchar(void)
{
	int c;

	if (!rx_packet)
	{
		while ( !(rx_packet = usb_rx(CDC_RX_ENDPOINT) ) || (rx_packet->index >= rx_packet->len) );
		rx_i=rx_packet->index;
		fastpath=rx_packet->len-2;
	}

	c = (uint8_t)rx_packet->buf[rx_i++];

	if (rx_i == rx_packet->len)
	{
		usb_free(rx_packet);
		rx_packet = NULL;
	}

	return c;
}

static int __attribute__ ((noinline)) gobble_badframe(int ff_count)
{
	int c;
	for(;;)
	{
		c = usb_getchar();

		if ( (ff_count == 3) && (c == 0xf0) )
			return 0;

		if (c == 0xff)
		{
			if (ff_count < 3 )
				ff_count++;
		}
		else
			ff_count=0;
	}
}

static int __attribute__ ((noinline)) gobble_endframe(void)
{
	int c = usb_getchar();

	if ( c == 0xff )
	{
		c = usb_getchar();
		if ( c == 0xf0 )
			return 1;
	}

	if ( c == 0xf0 )
		return 0; /* end of frame out of sync, re-sync, throw away frame */

	return gobble_badframe(c==0xff ? 3 : 0);
}

static int read_next_frame(void)
{
	int baud_ix=0, c;
	uint16_t strip_bit = 1;

	for(;;)
	{

		if (rx_i<fastpath)
		{
			uint8_t *p = &rx_packet->buf[rx_i];
			c = p[0] | (p[1]<<8);
			rx_i += 2;
		}
		else
		{
			c = usb_getchar();
			c |= usb_getchar()<<8;
		}

		if (c > 0xff00)
		{
			if (c == 0xffff)
				return gobble_endframe();
			else
				return gobble_badframe(1);
		}

		if ( strip_bit )
		{
			uint16_t *p = &next[baud_ix];

			if (c&0x0001) p[1] |= strip_bit;
			else          p[1] &= ~strip_bit;

			if (c&0x0002) p[2] |= strip_bit;
			else          p[2] &= ~strip_bit;

			if (c&0x0004) p[3] |= strip_bit;
			else          p[3] &= ~strip_bit;

			if (c&0x0008) p[4] |= strip_bit;
			else          p[4] &= ~strip_bit;

			if (c&0x0010) p[5] |= strip_bit;
			else          p[5] &= ~strip_bit;

			if (c&0x0020) p[6] |= strip_bit;
			else          p[6] &= ~strip_bit;

			if (c&0x0040) p[7] |= strip_bit;
			else          p[7] &= ~strip_bit;

			if (c&0x0080) p[8] |= strip_bit;
			else          p[8] &= ~strip_bit;


			if (c&0x0100) p[11] |= strip_bit;
			else          p[11] &= ~strip_bit;

			if (c&0x0200) p[12] |= strip_bit;
			else          p[12] &= ~strip_bit;

			if (c&0x0400) p[13] |= strip_bit;
			else          p[13] &= ~strip_bit;

			if (c&0x0800) p[14] |= strip_bit;
			else          p[14] &= ~strip_bit;

			if (c&0x1000) p[15] |= strip_bit;
			else          p[15] &= ~strip_bit;

			if (c&0x2000) p[16] |= strip_bit;
			else          p[16] &= ~strip_bit;

			if (c&0x4000) p[17] |= strip_bit;
			else          p[17] &= ~strip_bit;

			if (c&0x8000) p[18] |= strip_bit;
			else          p[18] &= ~strip_bit;

			baud_ix += 20;

			if (baud_ix >= N_DATA_BAUDS_PER_STRIP)
			{
				baud_ix = 0;
				strip_bit <<= 1;
			}
		}
	}
}

void setup(void)
{
	usb_init();
}

void loop(void)
{
	memset(buf_a, 0, sizeof(buf_a));
	memset(buf_b, 0, sizeof(buf_b));
	init_buf(buf_a);
	init_buf(buf_b);
	cur = buf_a;
	next = buf_b;
	uint16_t *tmp;

    hex = new HexSerialz(N_BAUDS_PER_STRIP*2, 1000000);
    hex->begin();

int i=0;
uint32_t t0, t, tmax=0;

    for (;;)
    {

t0=micros();

		read_next_frame();

t=micros()-t0;
if (t>tmax)
    tmax = t;
i++;
if (i==4000)
{
    Serial.println(tmax);
    i=0;
    tmax=0;
}
        hex->show(cur);
        tmp=cur;
		cur=next;
		next=tmp;
    }
}


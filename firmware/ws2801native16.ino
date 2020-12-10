/*
 * Drive 16 LED strips at once over USB serial using (almost) 16 bit to 8 bit temporal dithering
 *
 * Protocol: ( [16 bit brightness]* [ FF FF FF F0 ] )*
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

#include <HexWS2801z.h>
#include <usb_dev.h>

#define LED_VALUES 3
#define N_LEDS_PER_STRIP 25
#define N_BYTES_PER_STRIP (N_LEDS_PER_STRIP * LED_VALUES)
#define N_STRIPS 12
#define N_LEDS (N_LEDS_PER_STRIP * N_STRIPS)
#define N_VALUES (N_LEDS * LED_VALUES)

uint16_t io_buf1[N_VALUES];
uint16_t io_buf2[N_VALUES];
uint16_t io_buf3[N_VALUES];
uint8_t res[N_VALUES];
uint8_t buf1[N_BYTES_PER_STRIP*16];
uint8_t buf2[N_BYTES_PER_STRIP*16];
uint16_t *draw_buf, *in_buf, *unused_buf;
HexWS2801z *hex;

const uint16_t shuf[] = {
/*
 * for i in [21,22,1,19,15,16,17,18,2,6,7,8,9,5,3,52,53,54,50,51,4,12,13,14,10,11,0,23,24,20,34,30,44,35,36,37,38,39,40,49,45,46,47,48,41,59,55,56,57,58,42,26,27,28,29,25,43,31,32,33]:
 *     print ("\t"+''.join(str(i*15+j*3+rgb)+"," for j in range(5) for rgb in (1,0,2)))
 */
	316,315,317,319,318,320,322,321,323,325,324,326,328,327,329,
	331,330,332,334,333,335,337,336,338,340,339,341,343,342,344,
	16,15,17,19,18,20,22,21,23,25,24,26,28,27,29,
	286,285,287,289,288,290,292,291,293,295,294,296,298,297,299,
	226,225,227,229,228,230,232,231,233,235,234,236,238,237,239,
	241,240,242,244,243,245,247,246,248,250,249,251,253,252,254,
	256,255,257,259,258,260,262,261,263,265,264,266,268,267,269,
	271,270,272,274,273,275,277,276,278,280,279,281,283,282,284,
	31,30,32,34,33,35,37,36,38,40,39,41,43,42,44,
	91,90,92,94,93,95,97,96,98,100,99,101,103,102,104,
	106,105,107,109,108,110,112,111,113,115,114,116,118,117,119,
	121,120,122,124,123,125,127,126,128,130,129,131,133,132,134,
	136,135,137,139,138,140,142,141,143,145,144,146,148,147,149,
	76,75,77,79,78,80,82,81,83,85,84,86,88,87,89,
	46,45,47,49,48,50,52,51,53,55,54,56,58,57,59,
	781,780,782,784,783,785,787,786,788,790,789,791,793,792,794,
	796,795,797,799,798,800,802,801,803,805,804,806,808,807,809,
	811,810,812,814,813,815,817,816,818,820,819,821,823,822,824,
	751,750,752,754,753,755,757,756,758,760,759,761,763,762,764,
	766,765,767,769,768,770,772,771,773,775,774,776,778,777,779,
	61,60,62,64,63,65,67,66,68,70,69,71,73,72,74,
	181,180,182,184,183,185,187,186,188,190,189,191,193,192,194,
	196,195,197,199,198,200,202,201,203,205,204,206,208,207,209,
	211,210,212,214,213,215,217,216,218,220,219,221,223,222,224,
	151,150,152,154,153,155,157,156,158,160,159,161,163,162,164,
	166,165,167,169,168,170,172,171,173,175,174,176,178,177,179,
	1,0,2,4,3,5,7,6,8,10,9,11,13,12,14,
	346,345,347,349,348,350,352,351,353,355,354,356,358,357,359,
	361,360,362,364,363,365,367,366,368,370,369,371,373,372,374,
	301,300,302,304,303,305,307,306,308,310,309,311,313,312,314,
	511,510,512,514,513,515,517,516,518,520,519,521,523,522,524,
	451,450,452,454,453,455,457,456,458,460,459,461,463,462,464,
	661,660,662,664,663,665,667,666,668,670,669,671,673,672,674,
	526,525,527,529,528,530,532,531,533,535,534,536,538,537,539,
	541,540,542,544,543,545,547,546,548,550,549,551,553,552,554,
	556,555,557,559,558,560,562,561,563,565,564,566,568,567,569,
	571,570,572,574,573,575,577,576,578,580,579,581,583,582,584,
	586,585,587,589,588,590,592,591,593,595,594,596,598,597,599,
	601,600,602,604,603,605,607,606,608,610,609,611,613,612,614,
	736,735,737,739,738,740,742,741,743,745,744,746,748,747,749,
	676,675,677,679,678,680,682,681,683,685,684,686,688,687,689,
	691,690,692,694,693,695,697,696,698,700,699,701,703,702,704,
	706,705,707,709,708,710,712,711,713,715,714,716,718,717,719,
	721,720,722,724,723,725,727,726,728,730,729,731,733,732,734,
	616,615,617,619,618,620,622,621,623,625,624,626,628,627,629,
	886,885,887,889,888,890,892,891,893,895,894,896,898,897,899,
	826,825,827,829,828,830,832,831,833,835,834,836,838,837,839,
	841,840,842,844,843,845,847,846,848,850,849,851,853,852,854,
	856,855,857,859,858,860,862,861,863,865,864,866,868,867,869,
	871,870,872,874,873,875,877,876,878,880,879,881,883,882,884,
	631,630,632,634,633,635,637,636,638,640,639,641,643,642,644,
	391,390,392,394,393,395,397,396,398,400,399,401,403,402,404,
	406,405,407,409,408,410,412,411,413,415,414,416,418,417,419,
	421,420,422,424,423,425,427,426,428,430,429,431,433,432,434,
	436,435,437,439,438,440,442,441,443,445,444,446,448,447,449,
	376,375,377,379,378,380,382,381,383,385,384,386,388,387,389,
	646,645,647,649,648,650,652,651,653,655,654,656,658,657,659,
	466,465,467,469,468,470,472,471,473,475,474,476,478,477,479,
	481,480,482,484,483,485,487,486,488,490,489,491,493,492,494,
	496,495,497,499,498,500,502,501,503,505,504,506,508,507,509,
};

void scatter_bits(uint16_t *in, uint8_t *out)
{
	int i;
	uint32_t *o32 = (uint32_t *)out;

	/* copied from fadecandy, firmware/fc_draw.cpp & adapted
     * to scatter 16x 8 bits instead of 8x 24 bits
     * in this example, for ws2801 we only scatter 12 bits,
	 * leaving the other 4 pins for clock
	 */
	union
	{
		uint32_t word;
		struct
		{
			uint32_t
				clk0a:1, data0a:1, data1a:1, data2a:1, data3a:1, data4a :1, data5a :1, clk1a:1,
				clk2a:1, data6a:1, data7a:1, data8a:1, data9a:1, data10a:1, data11a:1, clk3a:1,
				clk0b:1, data0b:1, data1b:1, data2b:1, data3b:1, data4b :1, data5b :1, clk1b:1,
				clk2b:1, data6b:1, data7b:1, data8b:1, data9b:1, data10b:1, data11b:1, clk3b:1;
		};
	} o0, o1, o2, o3;

	o0.word = o1.word = o2.word = o3.word = 0;

	for (i=0; i<N_BYTES_PER_STRIP; i++)
	{
		uint32_t p0 = in[i]+res[i];
		res[i] = (uint8_t)p0;
		o3.data0b = p0 >> 8;
		o3.data0a = p0 >> 9;
		o2.data0b = p0 >> 10;
		o2.data0a = p0 >> 11;
		o1.data0b = p0 >> 12;
		o1.data0a = p0 >> 13;
		o0.data0b = p0 >> 14;
		o0.data0a = p0 >> 15;
		uint32_t p1 = in[i+N_BYTES_PER_STRIP]+res[i+N_BYTES_PER_STRIP];
		res[i+N_BYTES_PER_STRIP] = (uint8_t)p1;
		o3.data1b = p1 >> 8;
		o3.data1a = p1 >> 9;
		o2.data1b = p1 >> 10;
		o2.data1a = p1 >> 11;
		o1.data1b = p1 >> 12;
		o1.data1a = p1 >> 13;
		o0.data1b = p1 >> 14;
		o0.data1a = p1 >> 15;
		uint32_t p2 = in[i+N_BYTES_PER_STRIP*2]+res[i+N_BYTES_PER_STRIP*2];
		res[i+N_BYTES_PER_STRIP*2] = (uint8_t)p2;
		o3.data2b = p2 >> 8;
		o3.data2a = p2 >> 9;
		o2.data2b = p2 >> 10;
		o2.data2a = p2 >> 11;
		o1.data2b = p2 >> 12;
		o1.data2a = p2 >> 13;
		o0.data2b = p2 >> 14;
		o0.data2a = p2 >> 15;
		uint32_t p3 = in[i+N_BYTES_PER_STRIP*3]+res[i+N_BYTES_PER_STRIP*3];
		res[i+N_BYTES_PER_STRIP*3] = (uint8_t)p3;
		o3.data3b = p3 >> 8;
		o3.data3a = p3 >> 9;
		o2.data3b = p3 >> 10;
		o2.data3a = p3 >> 11;
		o1.data3b = p3 >> 12;
		o1.data3a = p3 >> 13;
		o0.data3b = p3 >> 14;
		o0.data3a = p3 >> 15;
		uint32_t p4 = in[i+N_BYTES_PER_STRIP*4]+res[i+N_BYTES_PER_STRIP*4];
		res[i+N_BYTES_PER_STRIP*4] = (uint8_t)p4;
		o3.data4b = p4 >> 8;
		o3.data4a = p4 >> 9;
		o2.data4b = p4 >> 10;
		o2.data4a = p4 >> 11;
		o1.data4b = p4 >> 12;
		o1.data4a = p4 >> 13;
		o0.data4b = p4 >> 14;
		o0.data4a = p4 >> 15;
		uint32_t p5 = in[i+N_BYTES_PER_STRIP*5]+res[i+N_BYTES_PER_STRIP*5];
		res[i+N_BYTES_PER_STRIP*5] = (uint8_t)p5;
		o3.data5b = p5 >> 8;
		o3.data5a = p5 >> 9;
		o2.data5b = p5 >> 10;
		o2.data5a = p5 >> 11;
		o1.data5b = p5 >> 12;
		o1.data5a = p5 >> 13;
		o0.data5b = p5 >> 14;
		o0.data5a = p5 >> 15;
		uint32_t p6 = in[i+N_BYTES_PER_STRIP*6]+res[i+N_BYTES_PER_STRIP*6];
		res[i+N_BYTES_PER_STRIP*6] = (uint8_t)p6;
		o3.data6b = p6 >> 8;
		o3.data6a = p6 >> 9;
		o2.data6b = p6 >> 10;
		o2.data6a = p6 >> 11;
		o1.data6b = p6 >> 12;
		o1.data6a = p6 >> 13;
		o0.data6b = p6 >> 14;
		o0.data6a = p6 >> 15;
		uint32_t p7 = in[i+N_BYTES_PER_STRIP*7]+res[i+N_BYTES_PER_STRIP*7];
		res[i+N_BYTES_PER_STRIP*7] = (uint8_t)p7;
		o3.data7b = p7 >> 8;
		o3.data7a = p7 >> 9;
		o2.data7b = p7 >> 10;
		o2.data7a = p7 >> 11;
		o1.data7b = p7 >> 12;
		o1.data7a = p7 >> 13;
		o0.data7b = p7 >> 14;
		o0.data7a = p7 >> 15;
		uint32_t p8 = in[i+N_BYTES_PER_STRIP*8]+res[i+N_BYTES_PER_STRIP*8];
		res[i+N_BYTES_PER_STRIP*8] = (uint8_t)p8;
		o3.data8b = p8 >> 8;
		o3.data8a = p8 >> 9;
		o2.data8b = p8 >> 10;
		o2.data8a = p8 >> 11;
		o1.data8b = p8 >> 12;
		o1.data8a = p8 >> 13;
		o0.data8b = p8 >> 14;
		o0.data8a = p8 >> 15;
		uint32_t p9 = in[i+N_BYTES_PER_STRIP*9]+res[i+N_BYTES_PER_STRIP*9];
		res[i+N_BYTES_PER_STRIP*9] = (uint8_t)p9;
		o3.data9b = p9 >> 8;
		o3.data9a = p9 >> 9;
		o2.data9b = p9 >> 10;
		o2.data9a = p9 >> 11;
		o1.data9b = p9 >> 12;
		o1.data9a = p9 >> 13;
		o0.data9b = p9 >> 14;
		o0.data9a = p9 >> 15;
		uint32_t p10 = in[i+N_BYTES_PER_STRIP*10]+res[i+N_BYTES_PER_STRIP*10];
		res[i+N_BYTES_PER_STRIP*10] = (uint8_t)p10;
		o3.data10b = p10 >> 8;
		o3.data10a = p10 >> 9;
		o2.data10b = p10 >> 10;
		o2.data10a = p10 >> 11;
		o1.data10b = p10 >> 12;
		o1.data10a = p10 >> 13;
		o0.data10b = p10 >> 14;
		o0.data10a = p10 >> 15;
		uint32_t p11 = in[i+N_BYTES_PER_STRIP*11]+res[i+N_BYTES_PER_STRIP*11];
		res[i+N_BYTES_PER_STRIP*11] = (uint8_t)p11;
		o3.data11b = p11 >> 8;
		o3.data11a = p11 >> 9;
		o2.data11b = p11 >> 10;
		o2.data11a = p11 >> 11;
		o1.data11b = p11 >> 12;
		o1.data11a = p11 >> 13;
		o0.data11b = p11 >> 14;
		o0.data11a = p11 >> 15;

		*(o32++) = o0.word;
		*(o32++) = o1.word;
		*(o32++) = o2.word;
		*(o32++) = o3.word;

		handle_io();
	}

}


unsigned int in_offset = 0;
unsigned int buf_align = 0;
int bad_frame = 0, end_frame = 0;
uint16_t c;

void swapbufs()
{
	if (!bad_frame)
	{
		uint16_t *x = draw_buf;
		draw_buf = in_buf;
		in_buf = unused_buf;
		unused_buf = x;
	}
	in_offset = 0;
	end_frame = 0;
	bad_frame = 0;
}


void handle_io()
{
	usb_packet_t *rx_packet = usb_rx(CDC_RX_ENDPOINT);
	if (!rx_packet)
		return;

	for (int i=rx_packet->index; i<rx_packet->len; i++)
	{
		if (buf_align)
		{
			c |= (uint8_t)rx_packet->buf[i] << 8;

			if (end_frame)
			{
				if (c == 0xf0ff)
					swapbufs();
				else if ( (c&0xff) == 0xf0 ) /* synchronize, throw away frame */
				{
					c = (uint8_t)rx_packet->buf[i];
					in_offset = 0;
					end_frame = 0;
					bad_frame = 0;
					continue;
				}
				else
				{
					bad_frame = 1;
					end_frame = (c == 0xffff);
				}
			}
			else if (c <= 0xff00)
			{
				if (in_offset < N_VALUES)
				{
					in_buf[shuf[in_offset]] = c;
					in_offset++;
				}
			}
			else if (c == 0xffff)
				end_frame = 1;
			else
				bad_frame = 1;
		}
		else
			c = (uint8_t)rx_packet->buf[i];

		buf_align ^= 1;
	}

	usb_free(rx_packet);
}

void setup()
{
	usb_init();
}

void loop()
{

	uint8_t *x, *old_frame = buf1, *new_frame = buf2;

	draw_buf = io_buf1;
	in_buf = io_buf2;
	unused_buf = io_buf3;
	memset(io_buf1, 0, sizeof(io_buf1));
	memset(io_buf2, 0, sizeof(io_buf2));
	memset(io_buf3, 0, sizeof(io_buf3));

	uint32_t j;
	for (j=0; j<N_BYTES;j++)
		res[j]=j*102;

    hex = new HexWS2801z(N_BYTES_PER_STRIP*16, 0x8181, 1, 1500000);
    hex->begin();

/*
int i=0;
uint32_t t0, t, tmax=0;
*/
    for (;;)
    {
/*
t0=micros();
*/
        scatter_bits(draw_buf, new_frame);
        hex->show(new_frame);
        x=old_frame;
        old_frame = new_frame;
        new_frame = x;
/*
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
*/
/*
int i=0;
uint32_t t0, t1=micros();
*/
    }
}


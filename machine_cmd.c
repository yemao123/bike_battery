#ifdef _WIN32
#include <Windows.h>
#else
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const unsigned short ccitt_table[256] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

unsigned short crc_ccitt(unsigned char *q, int len)
{
	unsigned short crc = 0;

	while (len-- > 0)
	crc = ccitt_table[(crc >> 8 ^ *q++) & 0xff] ^ (crc << 8);
	return ~crc;
}

const char CRC8Table[]={
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

unsigned char CRC8_Table(unsigned char *p, char counter)
{
    unsigned char crc8 = 0;
    for( ; counter > 0; counter--){
        crc8 = CRC8Table[crc8^*p];
        p++;
    }
    return crc8;
}
//#ifdef _WIN32
static unsigned char machine_cpu_key[12] = {0x38, 0xff, 0xD7, 0x05, 0x48, 0x59, 0x33, 0x33, 0x27, 0x84, 0x22, 0x43};
//#endif

static unsigned char char_tbl[] = {	'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
									'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
									'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
									'-', '+', '`', '~', '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '=', '{', '}', '[', ']', '|', ';', ':', '<', '>', ',', '.', '?',
									'\"', '\'', '/', '\\', ' ', '\t',
};

int encode_cmd_buffer(unsigned char *buf, unsigned char len)
{
	unsigned char save[128];
	unsigned char xor_len;
	unsigned char crc;
	int i, j;

	if(len > 61)
		return 0;

	memcpy(save, buf, len);
	buf[0] = char_tbl[rand() % (sizeof(char_tbl)-2)];
	buf[1] = char_tbl[rand() % (sizeof(char_tbl)-2)];
	xor_len = len ^ buf[0] ^ buf[1];
	for(i=0; i<sizeof(machine_cpu_key); i++) {
		xor_len ^= machine_cpu_key[i];
	}
	buf[2] = char_tbl[xor_len & 0x3f];
	buf[3] = char_tbl[(xor_len & 0xfc) >> 2];

	for(i=0; i<len; i++) {
		unsigned char tmp = save[i];
		tmp ^= buf[i%4];
		tmp ^= machine_cpu_key[(len-i-1)%sizeof(machine_cpu_key)];

		buf[4+i*2] = char_tbl[tmp & 0x3f];
		buf[5+i*2] = char_tbl[(tmp & 0xfc) >> 2];
	}

	for(j=4+len*2; j<126; j++) {
		buf[j] = char_tbl[rand() % (sizeof(char_tbl)-2)];
	}

	crc = CRC8_Table(buf, 126);
	buf[126] = char_tbl[crc & 0x3f];
	buf[127] = char_tbl[(crc & 0xfc) >> 2];

	return 128;
}

static int find_index(char ch)
{
	int i;
	for(i=0; i<sizeof(char_tbl); i++) {
		if(ch == char_tbl[i])
			return i;
	}
	return -1;
}

int decode_cmd_buffer(char *obuf, int len)
{
	unsigned char buf[128];
	unsigned char xor_len;
	unsigned char c2, c3, crc;
	int i;

	if(len != 128)
		return 0;

	memcpy(buf, obuf, len);

	c2 = find_index(buf[126]);
	c3 = find_index(buf[127]);
	crc = (c3 << 2) | (c2 & 3);
	if(crc != CRC8_Table(buf, 126)) {
		return 0;
	}

	c2 = find_index(buf[2]);
	c3 = find_index(buf[3]);

	xor_len = (c3 << 2) | (c2 & 3);
	for(i=0; i<sizeof(machine_cpu_key); i++) {
		xor_len ^= machine_cpu_key[i];
	}
	len = xor_len ^ buf[0] ^ buf[1];

	for(i=0; i<len; i++) {
		unsigned char tmp;
		c2 = find_index(buf[4+i*2]);
		c3 = find_index(buf[5+i*2]);
		tmp = (c3 << 2) | (c2 & 3);

		tmp ^= machine_cpu_key[(len-i-1)%sizeof(machine_cpu_key)];
		tmp ^= buf[i%4];
		obuf[i] = tmp;
	}
	return len;
}

#ifdef _WIN32
void machine_cmd_test(void)
{
	int i;
	srand(GetTickCount());
	while(1) {
		unsigned char buf[128+16];
		int len;
		strcpy(buf, "moto goto x:1234 y:6795 z:20355 r:4567 a:12314 b:122345 c:235");
		len = strlen(buf);

		len = encode_cmd_buffer(buf, len);
		buf[len] = 0;
		printf("encode(%d): %s\n", len, buf);
		for(i=0; i<len; i++) {
			if((i%16) == 0)
				printf("\n");
			printf("%02X ", buf[i]);
		}
		printf("\n");


		len = decode_cmd_buffer(buf, len);
		buf[len] = 0;
		printf("decode(%d): %s\n", len, buf);
		for(i=0; i<len; i++) {
			if((i%16) == 0)
				printf("\n");
			printf("%02X ", buf[i]);
		}
		printf("\n");
		getch();
	}
}

#endif
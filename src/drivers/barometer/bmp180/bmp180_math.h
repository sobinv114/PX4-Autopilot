/****************************************************************************
 * BMP180 calibration parsing and integer compensation.
 * Algorithm: Bosch BMP180 datasheet BST-BMP180-DS000-09 rev 2.5, figure 4.
 * Only depends on <stdint.h> so it can be unit tested on a PC.
 ****************************************************************************/

#pragma once

#include <stdint.h>

namespace Bosch_BMP180
{

static constexpr unsigned CALIB_SIZE = 22; // 11 words, registers 0xAA..0xBF, MSB first

struct calib_data {
	int16_t ac1, ac2, ac3;
	uint16_t ac4, ac5, ac6;
	int16_t b1, b2, mb, mc, md;
};

// DS 3.4: a word of 0x0000 or 0xFFFF means the EEPROM read failed.
inline bool parse_calibration(const uint8_t *buf, calib_data &c)
{
	uint16_t w[11];

	for (int i = 0; i < 11; i++) {
		w[i] = (uint16_t)(((uint16_t)buf[2 * i] << 8) | buf[2 * i + 1]);

		if (w[i] == 0x0000 || w[i] == 0xFFFF) {
			return false;
		}
	}

	c.ac1 = (int16_t)w[0];
	c.ac2 = (int16_t)w[1];
	c.ac3 = (int16_t)w[2];
	c.ac4 = w[3];
	c.ac5 = w[4];
	c.ac6 = w[5];
	c.b1 = (int16_t)w[6];
	c.b2 = (int16_t)w[7];
	c.mb = (int16_t)w[8];
	c.mc = (int16_t)w[9];
	c.md = (int16_t)w[10];
	return true;
}

// ut: raw 16 bit temperature. Outputs b5 (needed by the pressure formula) and temperature in 0.1 degC.
inline bool compensate_temperature(const calib_data &c, int32_t ut, int32_t &b5, int32_t &temperature_0p1c)
{
	const int32_t x1 = (int32_t)(((int64_t)(ut - (int32_t)c.ac6) * (int64_t)c.ac5) / 32768);
	const int32_t den = x1 + (int32_t)c.md;

	if (den == 0) {
		return false;
	}

	const int32_t x2 = ((int32_t)c.mc * 2048) / den;
	b5 = x1 + x2;
	temperature_0p1c = (b5 + 8) / 16;

	// operating range is -40..+85 degC, anything else is bus garbage
	return (temperature_0p1c > -500) && (temperature_0p1c < 1000);
}

// up: raw pressure already shifted right by (8 - oss). Output in Pa.
inline bool compensate_pressure(const calib_data &c, int32_t b5, int32_t up, uint8_t oss, int32_t &pressure_pa)
{
	if (oss > 3) {
		return false;
	}

	const int32_t b6 = b5 - 4000;
	const int32_t b6_sq = (b6 * b6) / 4096;

	int32_t x1 = ((int32_t)c.b2 * b6_sq) / 2048;
	int32_t x2 = ((int32_t)c.ac2 * b6) / 2048;
	int32_t x3 = x1 + x2;
	const int32_t b3 = ((((int32_t)c.ac1 * 4 + x3) * (1 << oss)) + 2) / 4;

	x1 = ((int32_t)c.ac3 * b6) / 8192;
	x2 = ((int32_t)c.b1 * b6_sq) / 65536;
	x3 = ((x1 + x2) + 2) / 4;

	if ((x3 + 32768) <= 0) {
		return false;
	}

	const uint32_t b4 = ((uint32_t)c.ac4 * (uint32_t)(x3 + 32768)) / 32768;

	if (b4 == 0) {
		return false;
	}

	const uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * (uint32_t)(50000 >> oss);

	int32_t p = (b7 < 0x80000000U) ? (int32_t)((b7 * 2) / b4) : (int32_t)((b7 / b4) * 2);

	if (p < 0 || p > 200000) {
		return false;
	}

	x1 = (p / 256) * (p / 256);
	x1 = (x1 * 3038) / 65536;
	x2 = (-7357 * p) / 65536;
	p = p + (x1 + x2 + 3791) / 16;

	pressure_pa = p;
	return true;
}

} // namespace Bosch_BMP180

#pragma once
#include "main.h"


typedef union{
	uint64_t data;
	uint8_t byte[5];
	struct __attribute__((__packed__)) {
		uint8_t cs;
		uint16_t tmp;
		uint16_t rh;
	}result;
}am_typedef;


void am_init(void);


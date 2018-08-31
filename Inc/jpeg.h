/*
 * jpeg.h
 *
 *  Created on: 30 de ago de 2018
 *      Author: Renan
 */

#ifndef JPEG_H_
#define JPEG_H_

#include "stm32f765xx.h"

#define JPEG_BASE 0x50051000

typedef struct {
__IO uint32_t JPEG_CONFR0;
__IO uint32_t JPEG_CONFR1;
__IO uint32_t JPEG_CONFR2;
__IO uint32_t JPEG_CONFR3;
__IO uint32_t JPEG_CONFR4;
__IO uint32_t JPEG_CONFR5;
__IO uint32_t JPEG_CONFR6;
__IO uint32_t JPEG_CONFR7;
__IO uint32_t RESERVED0;
__IO uint32_t RESERVED1;
__IO uint32_t RESERVED2;
__IO uint32_t RESERVED3;
__IO uint32_t JPEG_CR;
__IO uint32_t JPEG_SR;
__IO uint32_t JPEG_CFR;
__IO uint32_t JPEG_DIR;
__IO uint32_t JPEG_DOR;
__IO uint32_t QMEM;
__IO uint32_t HUFFMIN;
__IO uint32_t HUFFBASE;
__IO uint32_t HUFFSYMB;
__IO uint32_t DHTMEM;
__IO uint32_t HUFFENC;
} JPEG_Typedef;

#define JPEG ((JPEG_Typedef *) JPEG_BASE)

#endif /* JPEG_H_ */

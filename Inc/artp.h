/*
 * artp.h
 *
 *  Created on: 30 de set de 2018
 *      Author: renan
 */

#ifndef ARTP_H_
#define ARTP_H_

#include <stdint.h>

void draw_QRcode(uint8_t* image,int x_center,int y_center,int img_w,int img_h);

//tamanho da lado do quadrado em pixels
#define EDGE 24

#endif /* ARTP_H_ */

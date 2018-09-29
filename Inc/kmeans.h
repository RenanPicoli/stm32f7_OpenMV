/*
 * kmeans.h
 *
 *  Created on: 29 de set de 2018
 *      Author: renan
 */

#ifndef KMEANS_H_
#define KMEANS_H_

#define K_MEANS_OK 		0
#define K_MEANS_ERROR 	1

int k_means(int * * data, int n, int m, int k, float t, float * * centroids);

#endif /* KMEANS_H_ */

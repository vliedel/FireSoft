/**
 * @file CTime.h
 * @brief 
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2010 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    Oct 5, 2011
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    
 */


#ifndef CTIME_H_
#define CTIME_H_

#include <sys/time.h>
//#include <cstddef>

/**
 * Get the number of chunks of 1us from eternity to now. Get it twice
 * and you can subtract them from each other to calculate the time passed
 * in chunks of 1us.
 */
static long get_cur_1us() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000000 + tv.tv_usec);
}

/**
 * Get the number of chunks of 100us from eternity to now. Get it twice
 * and you can subtract them from each other to calculate the time passed
 * in chunks of 100us.
 */
static long get_cur_100us() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 10000 + tv.tv_usec / 100);
}

/**
 * Get the number of chunks of 1ms from eternity to now. Get it twice
 * and you can subtract them from each other to calculate the time passed
 * in chunks of 1ms (or 1000us).
 */
static long get_cur_1ms() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}


#endif /* CTIME_H_ */

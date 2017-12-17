#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <mraa.h>

uint32_t delta_ms; //Variable to store timestamp

unsigned int i; //Loop count variable

void main(void)
{

	struct timespec start, end;

	clock_gettime(CLOCK_MONOTONIC_RAW, &start);

	delta_ms = 0; //Initialize the timestamp

	for(i = 0; i < 10; i++)
	{
		printf("Elapsed time in milliseconds: %lu\n", delta_ms);

		sleep(1); //1 second delay

		clock_gettime(CLOCK_MONOTONIC_RAW, &end);

		delta_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
	}
}

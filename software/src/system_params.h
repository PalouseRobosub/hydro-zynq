#ifndef SYS_PARAMS_H
#define SYS_PARAMS_H

#include "types.h"

/*
 * System parameters.
 */
#define ARM_CLK_PLL 666667000

#define CPU_CLOCK_HZ (ARM_CLK_PLL/2)

#define FPGA_CLK 100000000

/*
 * UDP port definitions.
 */
#define COMMAND_SOCKET_PORT 3000

#define DATA_STREAM_PORT 3001
#define RESULT_PORT 3002
#define XCORR_STREAM_PORT 3003
#define DEBUG_PORT 3004
#define SILENT_REQUEST_PORT 3005

#define INITIAL_ADC_THRESHOLD 500

/**
 * Geometric constraints on the hydrophone sample shifts
 */
#define SPEED_SOUND_WATER_METERS_PER_SECOND 1498
#define HYDROPHONE_SPACING_METERS 0.02
#define MAX_TIME_BETWEEN_PHONES (HYDROPHONE_SPACING_METERS / SPEED_SOUND_WATER_METERS_PER_SECOND)

/**
 * Defines the maximum samples that can occur between two hydrophones at a sampling rate of 5Msps.
 */
#define MAX_SAMPLES_BETWEEN_PHONES ((MAX_TIME_BETWEEN_PHONES * 5000000))

#endif

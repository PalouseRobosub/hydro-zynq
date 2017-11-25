#ifndef SYS_PARAMS_H
#define SYS_PARAMS_H

/*
 * System parameters.
 */
#define ADC_THRESHOLD 0x100

#define ARM_CLK_PLL 666667000

#define CPU_CLOCK_HZ (ARM_CLK_PLL/2)

#define SAMPLING_FREQUENCY 5000000

#define MAX_SAMPLES (SAMPLING_FREQUENCY/1000 * 2100)

#define SAMPLES_PER_PACKET 128

/*
 * UDP port definitions.
 */
#define COMMAND_SOCKET_PORT 3000
#define DATA_STREAM_PORT 3001
#define RESULT_PORT 3002

#endif

#include "adc_dma_addresses.h"

#include "regs/AdcRegs.h"
#include "regs/DmaRegs.h"
#include "regs/SpiRegs.h"
#include "types.h"

struct AdcRegs adc_regs = {
    .clk_div = 10,
    .samples_per_packet = 256
};

struct SpiRegs spi_regs;

struct DmaRegs dma_regs;

uint32_t ADC_BASE_ADDRESS = (uint32_t) &adc_regs;

uint32_t SPI_BASE_ADDRESS = (uint32_t) &spi_regs;

uint32_t DMA_BASE_ADDRESS = (uint32_t) &dma_regs;

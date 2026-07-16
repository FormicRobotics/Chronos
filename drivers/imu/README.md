# ICM-42688-P IMU driver — NOT used on Chronos R1

On the Chronos board (HW_CHRONOS_R1) the ICM-42688-P IMU (IC1) is wired
**only to the FPGA**:

- SPI (CS / SCLK / SDI / SO) goes to CrossLink-NX pins.
- INT1 goes to FPGA pin Y7.
- FSYNC is driven by the FPGA with the same edge that triggers the cameras.

The Jetson has **no SPI or GPIO path** to this device. This Jetson-side SPI
driver therefore cannot bind to real hardware on Chronos R1, there is no IMU
node in `drivers/chronos-orin-nx.dts`, and the module **must not be loaded**
on this board.

## Why is the code still here?

It is retained for two possible futures:

1. A hardware revision that routes the IMU SPI/INT to the Jetson — this
   driver would then work as-is with a proper `spi` DT node.
2. An FPGA-side SPI master (currently future work in the RTL) that proxies
   IMU samples through the FPGA's I2C register file at 0x3C. In that case
   the register access layer here serves as a reference for the sample
   decoding and FSYNC-tag handling, but the transport would have to be
   rewritten against the `chronos_fpga` driver.

## Building

Excluded from the top-level `drivers/Makefile` on purpose. If you need it
for a future board revision:

```bash
cd drivers/imu
make KDIR=/path/to/kernel/build
```

# Chronos FPGA - MIPI D-PHY IP

The MIPI D-PHY is provided by Lattice's `lscc_mipi` reference IP. The Chronos
RTL does not vendor a copy of that licensed RTL; instead the thin wrappers in
`fpga/rtl/mipi/` (`mipi_dphy_rx.v`, `mipi_dphy_tx.v`) instantiate the Lattice
modules `lscc_mipi_wrapper_rx` / `lscc_mipi_wrapper_tx`.

## Making the IP available to the Radiant 2.0 SP1 project

Pick ONE of the following:

### Option A - IP Catalog (recommended, gives correct TX PLL coefficients)
1. Radiant 2.0 SP1 -> IP Catalog -> IP -> `mipi_dphy`.
2. Generate two configurations into `fpga/ip/`:
   - `mipi_dphy_rx_soft`: Interface CSI-2, **RX**, **Soft (LATTICE/GDDR)**,
     NUM_LANE = 2, GEAR = 8, line rate 800 Mbps, continuous clock.
   - `mipi_dphy_tx_hard`: Interface CSI-2, **TX**, **Hard**, NUM_LANE = 2,
     GEAR = 8, line rate 800 Mbps, internal PLL, ref clock = clk_sys frequency.
3. From the generated TX core, copy the emitted `CM`/`CN`/`CO` and
   `REF_CLOCK_FREQ` into the `mipi_dphy_tx` instantiation in `chronos_top.sv`.
4. Add the generated IP source list (.ipx) to the project.

### Option B - reference RTL search path (quick bring-up)
Add the install IP RTL folder to the project source search path so the
`lscc_*` modules resolve:

```
C:/lscc/radiant/2.0/ip/lifcl/mipi_dphy/rtl
```

(Add every `.v` in that folder to the project, or add the folder as an include/
search path.) With Option B you still must determine the hard-TX PLL `CM/CN/CO`
for the target line rate; Option A computes them for you.

## Notes
- Soft RX provides no SoT/byte-enable; `csi2_rx` does fabric word alignment
  (0xB8 detection). See `fpga/DESIGN_DECISIONS.md`.
- Build with Synplify Pro synthesis (SystemVerilog), Radiant 2.0 SP1, ES device.

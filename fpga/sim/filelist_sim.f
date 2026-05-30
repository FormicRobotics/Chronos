# Simulation file list for the CSI-2 datapath loopback testbench.
# IMPORTANT: this intentionally EXCLUDES fpga/rtl/mipi/* - the behavioral
# models in mipi_dphy_models.sv replace mipi_dphy_rx / mipi_dphy_tx so the
# packet path can be simulated without the Lattice MIPI primitives.
#
#   Questa/ModelSim:  vlog -sv -f fpga/sim/filelist_sim.f && vsim -c tb_chronos_csi2 -do "run -all; quit"
#   Icarus (>=12):    iverilog -g2012 -o sim.out -c fpga/sim/filelist_sim.f && vvp sim.out

../rtl/csi2_pkg.sv
mipi_dphy_models.sv
../rtl/sync_fifo.sv
../rtl/csi2_rx.sv
../rtl/frame_buffer.sv
../rtl/tx_arbiter.sv
../rtl/csi2_tx.sv
tb_chronos_csi2.sv

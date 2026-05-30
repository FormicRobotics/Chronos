#==========================================================================
# Lattice Radiant TCL build script for Chronos blinky design
#
# Target chip on the Chronos R1 board: Lattice CrossLink-NX LIFCL-40-ES
# (Engineering Sample variant, JTAG IDCODE 0x010F1043).
#
# IMPORTANT: Radiant Design Tool does NOT expose the ES variant.  It
# generates bitstreams with the production IDCODE (0x110F1043).  This is
# fine - you just need to configure Radiant *Programmer* correctly:
#
#   Device Properties:
#     Family          : LIFCL_ENG          <-- NOT "LIFCL"!
#     Device          : LIFCL-40-ES
#     Operation       : SRAM -> Fast Configuration (or Erase,Program,Verify)
#     Port Interface  : JTAG               <-- NOT "Slave SPI"
#     TCK Divider     : 8 or higher        (slow down for reliable ES op)
#     File            : impl1\chronos_blinky_impl1.bit
#
# The Programmer's LIFCL_ENG family handles the IDCODE difference
# transparently: it accepts production-IDCODE bitstreams on ES silicon.
#
# Run from this directory:
#   pnmainc.exe build_blinky.tcl
#==========================================================================

set SCRIPT_DIR   [file dirname [file normalize [info script]]]
cd $SCRIPT_DIR

set PROJ_NAME    "chronos_blinky"
set IMPL_NAME    "impl1"
set DEV_NAME     "LIFCL-40-9BG400C"
set TOP_MODULE   "chronos_blinky"
set SRC_FILE     "chronos_blinky.sv"
set PDC_FILE     "chronos_blinky.pdc"

puts "============================================================"
puts " Chronos blinky build for LIFCL-40 (-ES silicon on board)"
puts " Script dir : $SCRIPT_DIR"
puts " Project    : $PROJ_NAME / $IMPL_NAME"
puts " Device     : $DEV_NAME  (production part; ES will be"
puts "                          handled by Programmer LIFCL_ENG family)"
puts "============================================================"

if {[file exists "$PROJ_NAME.rdf"]} {
    puts "---- Opening existing project ----"
    prj_open "$PROJ_NAME.rdf"
} else {
    puts "---- Creating new project ----"
    prj_create -name $PROJ_NAME \
               -impl $IMPL_NAME \
               -dev  $DEV_NAME \
               -synthesis "lse"
}

puts "---- Adding sources ----"
if {[catch {prj_add_source $SRC_FILE -work work} err]} { puts "NOTE: src add: $err" }
if {[catch {prj_add_source $PDC_FILE -work work} err]} { puts "NOTE: pdc add: $err" }
if {[catch {prj_set_strategy_value -strategy Strategy1 "syn_top=$TOP_MODULE"} err]} {
    puts "NOTE: set top: $err"
}

puts "---- Synthesis ----"
prj_run Synthesis -impl $IMPL_NAME

puts "---- Map ----"
prj_run Map -impl $IMPL_NAME

puts "---- Place & Route ----"
prj_run PAR -impl $IMPL_NAME

puts "---- Bitstream Export ----"
if {[catch {prj_run Export -impl $IMPL_NAME -task Bitgen} err]} {
    puts "Bitgen task name failed ($err); trying plain Export..."
    prj_run Export -impl $IMPL_NAME
}

prj_save
prj_close

set BIT_FILE "$SCRIPT_DIR\\${IMPL_NAME}\\${PROJ_NAME}_${IMPL_NAME}.bit"
if {[file exists $BIT_FILE]} {
    puts ""
    puts "============================================================"
    puts " BUILD COMPLETE"
    puts " Bitstream: $BIT_FILE"
    puts ""
    puts " Program with Radiant Programmer:"
    puts "   Family          : LIFCL_ENG   <-- CRITICAL!  not LIFCL"
    puts "   Device          : LIFCL-40-ES"
    puts "   Operation       : SRAM -> Fast Configuration"
    puts "   Port Interface  : JTAG        <-- NOT Slave SPI"
    puts "   TCK Divider     : 8 (or higher) for reliable ES programming"
    puts "   File            : $BIT_FILE"
    puts "============================================================"
} else {
    puts "WARNING: bitstream not found at $BIT_FILE"
}

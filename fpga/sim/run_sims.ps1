# Runs all Chronos FPGA testbenches with Icarus Verilog (OSS CAD Suite).
# Usage:  powershell -File run_sims.ps1   (from fpga/sim)
# Requires iverilog/vvp on PATH; OSS CAD Suite default location is added below.

$ErrorActionPreference = "Stop"
$oss = Join-Path $env:LOCALAPPDATA "oss-cad-suite\bin"
if (Test-Path $oss) { $env:PATH = "$oss;$env:PATH" }

$fail = 0

Write-Host "=== tb_chronos_csi2 (CSI-2 datapath loopback) ==="
iverilog -g2012 -o sim.out -c filelist_sim.f
$out = vvp sim.out
$out | Write-Host
if (($out | Out-String) -notmatch "RESULT: PASS") { $fail++ }

Write-Host "=== tb_i2c_slave (host I2C BFM vs i2c_slave+config_regs) ==="
iverilog -g2012 -o i2c.out ../rtl/i2c_slave.sv ../rtl/config_regs.sv tb_i2c_slave.sv
$out = vvp i2c.out
$out | Write-Host
if (($out | Out-String) -notmatch "RESULT: PASS") { $fail++ }

Write-Host "=== tb_trigger_generator (phase-accumulator rate engine) ==="
iverilog -g2012 -o trig.out ../rtl/trigger_generator.sv tb_trigger_generator.sv
$out = vvp trig.out
$out | Write-Host
if (($out | Out-String) -notmatch "RESULT: PASS") { $fail++ }

Remove-Item -ErrorAction SilentlyContinue sim.out, i2c.out, trig.out

if ($fail -eq 0) { Write-Host "`nALL TESTBENCHES PASS" -ForegroundColor Green; exit 0 }
else             { Write-Host "`n$fail TESTBENCH(ES) FAILED" -ForegroundColor Red; exit 1 }


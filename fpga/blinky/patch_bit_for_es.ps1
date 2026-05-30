#==========================================================================
# Patch a Lattice CrossLink-NX .bit file for use on an LIFCL-40-ES chip.
#
# The Radiant Design Tool always embeds the production IDCODE (0x110F1043)
# into the bitstream's VERIFY_ID command, even when bitgen -g ES:Yes is
# used (that flag only updates the ASCII header text).  When loaded onto
# an Engineering Sample chip (IDCODE 0x010F1043), the FPGA's VERIFY_ID
# command fails and configuration aborts.
#
# This script:
#   1. Reads the original .bit file.
#   2. Finds the embedded VERIFY_ID IDCODE bytes (0x11 0x0F 0x10 0x43).
#   3. Patches byte 0 of the IDCODE from 0x11 to 0x01 -> matches ES.
#   4. Recomputes the "Bitstream CRC" stored in the ASCII header using
#      Lattice's CRC-16 algorithm (poly=0x8005, init=0x0000, reflectOut=True).
#   5. Updates the header "Bitstream CRC: 0xNNNN" field in-place so that
#      the Radiant Programmer's pre-flight integrity check passes.
#   6. Writes the patched bitstream to <inputname>_ES_fixed.bit
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File patch_bit_for_es.ps1 <input.bit>
#==========================================================================

param(
    [Parameter(Mandatory=$true, Position=0)]
    [string]$InputBit
)

if (-not (Test-Path -LiteralPath $InputBit)) {
    Write-Error "Input file not found: $InputBit"
    exit 1
}

$bytes = [System.IO.File]::ReadAllBytes($InputBit)
$origSize = $bytes.Length
Write-Host "Input  : $InputBit"
Write-Host "Size   : $origSize bytes"

#--------------------------------------------------------------------------
# Locate the binary section (starts at sync pattern 0xFFFFBDB3)
#--------------------------------------------------------------------------
$binStart = -1
for ($i = 0; $i -lt 1024; $i++) {
    if ($bytes[$i]   -eq 0xFF -and $bytes[$i+1] -eq 0xFF -and
        $bytes[$i+2] -eq 0xBD -and $bytes[$i+3] -eq 0xB3) {
        $binStart = $i; break
    }
}
if ($binStart -lt 0) { Write-Error "Could not find binary section sync pattern"; exit 1 }
Write-Host ('Binary : starts at 0x{0:X}, length {1}' -f $binStart, ($bytes.Length - $binStart))

#--------------------------------------------------------------------------
# Locate the VERIFY_ID command's IDCODE bytes (0x11 0x0F 0x10 0x43)
#--------------------------------------------------------------------------
$idcodeOff = -1
for ($i = $binStart; $i -lt $binStart + 1024; $i++) {
    if ($bytes[$i]   -eq 0x11 -and $bytes[$i+1] -eq 0x0F -and
        $bytes[$i+2] -eq 0x10 -and $bytes[$i+3] -eq 0x43) {
        # Verify the preceding 4 bytes look like VERIFY_ID (0xE2 opcode)
        if ($bytes[$i-4] -eq 0xE2) { $idcodeOff = $i; break }
    }
}
if ($idcodeOff -lt 0) {
    Write-Error "Could not find embedded production IDCODE 0x110F1043 preceded by VERIFY_ID opcode"
    exit 1
}
Write-Host ('IDCODE : at 0x{0:X} (was 0x110F1043 -> patching to 0x010F1043)' -f $idcodeOff)

# Apply IDCODE patch
$bytes[$idcodeOff] = 0x01

#--------------------------------------------------------------------------
# Recompute the Bitstream CRC over the binary section
#   poly=0x8005, init=0x0000, refIn=False, refOut=True, xorOut=0x0000
#--------------------------------------------------------------------------
function Compute-LatticeBitCRC([byte[]]$data, [int]$start, [int]$len) {
    $crc = [uint32]0
    for ($i = $start; $i -lt ($start + $len); $i++) {
        $crc = $crc -bxor ([uint32]$data[$i] -shl 8)
        for ($j = 0; $j -lt 8; $j++) {
            if (($crc -band 0x8000) -ne 0) { $crc = (($crc -shl 1) -bxor 0x8005) -band 0xFFFF }
            else { $crc = ($crc -shl 1) -band 0xFFFF }
        }
    }
    # Reflect output (16-bit)
    $r = [uint32]0
    for ($i = 0; $i -lt 16; $i++) {
        if ($crc -band (1 -shl $i)) { $r = $r -bor (1 -shl (15 - $i)) }
    }
    return $r -band 0xFFFF
}

$newCRC = Compute-LatticeBitCRC $bytes $binStart ($bytes.Length - $binStart)
Write-Host ('CRC    : recomputed = 0x{0:X4}' -f $newCRC)

#--------------------------------------------------------------------------
# Find and replace the "Bitstream CRC: 0xNNNN" text in the ASCII header.
# The header is the first $binStart bytes; the text uses exactly 4 hex digits.
#--------------------------------------------------------------------------
$header = [System.Text.Encoding]::ASCII.GetString($bytes, 0, $binStart)
$match  = [regex]::Match($header, "Bitstream CRC:\s*0x([0-9A-Fa-f]{4})")
if (-not $match.Success) { Write-Error "Could not find 'Bitstream CRC: 0xNNNN' in header"; exit 1 }
$oldCRC      = $match.Groups[1].Value.ToUpper()
$oldCRCStart = $match.Groups[1].Index
$newCRCText  = '{0:X4}' -f $newCRC

Write-Host ("CRC    : header was 0x{0}, replacing with 0x{1} at byte offset {2}" -f $oldCRC, $newCRCText, $oldCRCStart)

# Patch in-place (same byte length, so size unchanged)
$asciiNew = [System.Text.Encoding]::ASCII.GetBytes($newCRCText)
for ($i = 0; $i -lt 4; $i++) { $bytes[$oldCRCStart + $i] = $asciiNew[$i] }

#--------------------------------------------------------------------------
# Write output
#--------------------------------------------------------------------------
$dir  = Split-Path -Parent $InputBit
$name = [System.IO.Path]::GetFileNameWithoutExtension($InputBit)
$out  = Join-Path $dir ($name + '_ES_fixed.bit')
[System.IO.File]::WriteAllBytes($out, $bytes)

Write-Host ""
Write-Host "Output : $out"
Write-Host "Size   : $((Get-Item $out).Length) bytes (must match input: $origSize)"
Write-Host ""
Write-Host "Now program this file in Radiant Programmer:"
Write-Host "  Device Family   : LIFCL_ENG"
Write-Host "  Device          : LIFCL-40-ES"
Write-Host "  Port Interface  : Slave SPI  (whatever Display ID worked with)"
Write-Host "  Operation       : Fast Configuration"
Write-Host "  File Name       : $out"

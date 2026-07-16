# Fill zones and save (run with KiCad's bundled python)
import pcbnew

p = r"C:\Projects\Github\Chronos\hardware\adapter_chronos_helios\chronos_helios_adapter.kicad_pcb"
b = pcbnew.LoadBoard(p)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(p, b)
print("zones filled + saved")

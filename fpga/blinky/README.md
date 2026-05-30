# Chronos Blinky — FPGA Bring-up Sanity Check

Bu klasör, HW_CHRONOS_R1 kartı üzerindeki Lattice **CrossLink-NX (LIFCL-40)** FPGA
için **mümkün olan en küçük bitstream**'i içerir.  Tek amacı: Radiant toolchain'in
bitstream üretebildiğini, FT2232 üzerinden JTAG ile yüklediğimiz bitstream'in
gerçekten FPGA'da çalıştığını, ve dört yeşil LED'in mantıksal kontrol altında
olduğunu kanıtlamak.

## İçerik

| Dosya | Açıklama |
|---|---|
| `chronos_blinky.sv` | RTL — internal HFOSC + 28-bit sayaç + LED çıkışları |
| `chronos_blinky.pdc` | Pin constraint — 4 LED + GSRN reset |
| `build_blinky.tcl` | Radiant otomatik build scripti |

## Beklenen sonuç

Bitstream yüklendiğinde:
- **D14, D15, D16, D17** yeşil LED'leri bir ikili sayım deseninde yanıp söner
- En hızlısı ~0.4 s, en yavaşı ~3 s'de bir toggle yapar
- **SW4** (GSRN) butonuna basınca sayaç sıfırlanır → tüm LED'ler söner
- Bırakınca sayım yeniden başlar

## Çalıştırma — iki yol

### Yol 1: Otomatik (önerilen)

PowerShell veya cmd:

```powershell
cd C:\Projects\Github\Chronos\fpga\blinky
& "C:\lscc\radiant\<sürüm>\bin\nt64\pnmainc.exe" build_blinky.tcl
```

`<sürüm>` yerine kurulu Radiant sürümünü yaz (örn. `2024.2`).
Script bitince `impl1\chronos_blinky_impl1.bit` üretilmiş olur.

### Yol 2: GUI üzerinden

1. **Radiant Design Tool**'u (Programmer değil) aç.
2. **File → New → Project**:
   - Project name: `chronos_blinky`
   - Location: `C:\Projects\Github\Chronos\fpga\blinky`
   - Implementation: `impl1`
3. Device:
   - Family: **LIFCL (CrossLink-NX)**
   - Device: **LIFCL-40**
   - Package: **CABGA400** (BG400)
   - Speed: any (e.g. `-9`)
   - Operating: Industrial veya Commercial
4. Add Source:
   - `chronos_blinky.sv`
5. Add Constraint:
   - `chronos_blinky.pdc`
6. Sol panelde **Process** sekmesi → şu sırada çalıştır:
   - Synthesize Design
   - Map Design
   - Place & Route Design
   - Export Files → Bitstream File

## Bitstream'i yükleme — KRİTİK ES (Engineering Sample) AYARLARI

Chronos R1 kartındaki LIFCL-40 chip **Engineering Sample** (ES) — JTAG IDCODE
`0x010F1043`.  Radiant Design Tool ise bitstream'i her zaman üretim IDCODE'u
(`0x110F1043`) ile üretir — bu **normaldir**.  Programmer'ı doğru aileye
ayarlarsanız, üretim-IDCODE bitstream'i ES chip'i üzerinde sorunsuz çalışır.

### Programmer ayarları (sırayla)

1. **Radiant Programmer**'ı kapat, tekrar aç. **Detect Cable** → `HW-USBN-2B (FTDI)`,
   Port `FTUSB-0`.
2. Cihaz satırını çift tıkla → **Device Properties** aç.
3. **Device** sekmesinde:
   - **Device Family**: `LIFCL_ENG`        ← **MUTLAKA bu**, asla `LIFCL` değil!
   - **Device**: `LIFCL-40-ES`
4. **General** sekmesinde:
   - **Access Mode**: `Direct Programming`
   - **Operation**: `Fast Configuration`   (SRAM'e yükler, test için ideal)
   - **Port Interface**: `JTAG`            ← **NOT** `Slave SPI`
   - **File Name**: `impl1\chronos_blinky_impl1.bit`
5. **OK** → tabloda satır güncellensin.
6. Üst menü → **Edit → Cable and I/O Port Settings** (veya cable settings ikonu):
   - **TCK Divider**: `8`  (ya da daha yüksek — ES için daha güvenli)
7. **Run → Program** (yeşil ok).

Beklenen log:

```
INFO  - Operation: SRAM Erase, Program, Verify
INFO  - Device1 LIFCL-40-ES: Operation Done.
INFO  - Elapsed Time = ...
INFO  - All Operation Done. No errors.
```

Aynı anda:
- **D17 (kırmızı INITN)** söner
- **D18 (yeşil DONE)** yanar  ← config başarılı sinyali
- **D14..D17 yeşil LED'ler** ikili sayım yapmaya başlar

> Lattice resmi FAQ (FPGA-AN-02048): _"How to solve error in programming ES?
> Use 'LIFCL_ENG' as the device family instead of 'LIFCL'. This solves any
> programming issue. Also, vary the TCK divider to slow down the programming
> speed."_

## Sorun giderme

| Belirti | Olası neden | Çözüm |
|---|---|---|
| `Mismatch to the device ID code. Current: 0x010F1043, Expected: 0x110F1043` | Programmer'da Family `LIFCL` seçili | Family'i **`LIFCL_ENG`** yap |
| `Failed to read the Device's IDCODE` | JTAG TAP stuck | 12 V'yi çek-tak (USB'yi de) → Programmer'ı yeniden aç → Detect Cable |
| `File is invalid for the expected device` | .bit dosyası manuel modifiye edilmiş / yanlış part | Fresh build üret (build_blinky.tcl), patched veya eski .bit kullanma |
| `Programming failed` (uzun süre takılır) | TCK çok hızlı | TCK Divider'ı 8 → 16 → 30 dene |
| DONE LED yanmıyor, INITN açık kalıyor | Bitstream config tamamlanamadı | TCK Divider artır, Port Interface JTAG mı kontrol et, J44 jumper 1-2 pinde mi |
| Tüm LED'ler statik (sayım yok) | OSC_CORE primitive hatası | Radiant log'da synth uyarılarına bak, OSC çalıştığını HFOSC log'unda doğrula |
| `OSC_CORE` undefined | Eski Radiant sürümü | IP Catalog'tan PMI_OSC üret veya OSCA primitive'ini dene |
| Program başarılı ama LED yanmıyor | LED polarite ters / yanlış pin | PDC'de E17/F13/G13/F14 doğru, `IO_TYPE=LVCMOS33` ayarı doğru mu |

## Sonraki adım

Blinky çalıştığında:
1. Tam Chronos tasarımına geç (`fpga/rtl/`)
2. PLL'i Radiant IP Catalog'tan üret (PLL_CORE wrapper)
3. D-PHY hard IP'sini IP Catalog'tan üret
4. PDC SDC clock path'lerini düzelt
5. Sentez → P&R → Bitstream
6. Aynı şekilde yükle

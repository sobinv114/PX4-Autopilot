
MEMORY
{
  tcm_idram_seg (RX) : org = 0x30100000, len = 0x2000
  irom_seg (RX) : org = 0x40000000, len = (0x10000 << 10)
  sram_low (RWX) : org = 0x4FF00000, len = 0x4FF2CBD0 - 0x4FF00000
  sram_high (RW) : org = 0x4FF40000, len = 0x80000 - 0x20000
  drom_seg (R) : org = 0x40000000, len = (0x10000 << 10)
  rev3_mspi_workaround_seg(RWX) : org = 0x50108000, len = 0x0
  lp_ram_seg(RW) : org = 0x50108000 + 0x0 + (0 + (24)), len = 0x8000 - (0 + (24)) - 0x0
  lp_reserved_seg(RW) : org = 0x50108000 + 0x0, len = (0 + (24))
  extern_ram_seg(RWX) : org = 0x48000000, len = (0x10000 << 10)
}
_data_seg_org = ORIGIN(rtc_data_seg);
REGION_ALIAS("rtc_iram_seg", lp_ram_seg );
REGION_ALIAS("rtc_data_seg", rtc_iram_seg );
REGION_ALIAS("rtc_slow_seg", rtc_iram_seg );
REGION_ALIAS("rtc_data_location", rtc_iram_seg );
REGION_ALIAS("rtc_reserved_seg", lp_reserved_seg );
  REGION_ALIAS("text_seg_low", irom_seg);
  REGION_ALIAS("rodata_seg_low", drom_seg);
  REGION_ALIAS("ext_ram_seg", extern_ram_seg);
  ASSERT(_flash_rodata_dummy_start == ORIGIN(rodata_seg_low),
         ".flash_rodata_dummy section must be placed at the beginning of the rodata segment.")

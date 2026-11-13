// Thạch anh 27 MHz; Gowin tự suy ra xung nhịp 54 MHz ở đầu ra rPLL.
create_clock -name clk27 -period 37.037 -waveform {0 18.518} [get_ports {clk}]
// SPI là tín hiệu không đồng bộ (đã đồng bộ 2 tầng trong spi_frame_rx)
set_false_path -from [get_ports {spi_sck spi_mosi spi_cs_n}]

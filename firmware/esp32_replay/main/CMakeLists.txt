# Dùng CHUNG preproc.c và spi_link.c với firmware demo: cùng mã Gauss, cùng giao thức SPI
idf_component_register(SRCS "app_main.c" "../../esp32_demo/main/preproc.c" "../../esp32_demo/main/spi_link.c"
                       INCLUDE_DIRS "." "../../esp32_demo/main")

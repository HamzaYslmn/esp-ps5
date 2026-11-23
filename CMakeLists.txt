# ESP-IDF component (Arduino as a component). Arduino IDE / PlatformIO ignore this file.
idf_component_register(SRCS "src/ps5Controller.cpp" "src/ps5_bytes.cpp" "src/bluedroid/bluedroid.cpp"
                       INCLUDE_DIRS "src"
                       REQUIRES arduino-esp32 bt)

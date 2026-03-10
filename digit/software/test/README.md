ble_test: This folder has two files that we can use to test the esp32 ble.
    -> esp32_ble_test/esp32_ble_test.ino : This is the code to be uploaded to the ESP32
    -> esp32_ble_test/esp32_ble_test.py : This is to be used on the laptop/personal device

The main goal of this test is to send a message from the laptop/personal device to the esp32 and receive an ACK for the same.

Notes:
    On the Arduino IDE turn on USB CDC to be able to see output on the Serial terminal
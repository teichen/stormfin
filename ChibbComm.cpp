#include <iostream>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int main() {
    int fd;
    const char* device = "/dev/ttyACM0"; // Typical for Arduino Uno/Mega via USB
    int baud = 9600;

    // Open and initialize serial port
    if ((fd = serialOpen(device, baud)) < 0) {
        fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
        return 1;
    }

    // Initialize wiringPi (required for timing functions like delay)
    if (wiringPiSetup() == -1) {
        fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
        return 1;
    }

    std::cout << "Serial port opened successfully!" << std::endl;

    while (true) {
        // Send a message to the Arduino
        serialPuts(fd, "Hello Arduino\n");

        // Wait for and print response from Arduino
        while (serialDataAvail(fd)) {
            std::cout << (char)serialGetchar(fd);
            fflush(stdout);
        }

        delay(1000); // Wait 1 second
    }

    serialClose(fd);
    return 0;
}

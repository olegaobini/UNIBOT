#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// Function to get keyboard input without Enter key
char getch() {
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    return buf;
}

int main() {
    int pwmPin = 1; // GPIO18 (Pin 12)

    if (wiringPiSetup() == -1) {
        std::cout << "WiringPi setup failed!" << std::endl;
        return 1;
    }

    int frequency = 200; // Hz (initial frequency)
    int dutyCycle = 50;  // % (initial duty cycle)

    softPwmCreate(pwmPin, dutyCycle, 100);

    std::cout << "Controls: 'q'/'a' freq +/-50Hz | 'p'/'l' duty +/-5% | 'x' to exit\n";

    bool running = true;
    while (running) {
        char key = getch();
        switch (key) {
            case 'q':
                frequency += 50;
                if (frequency > 1000) frequency = 1000;
                std::cout << "Frequency: " << frequency << " Hz\n";
                break;
            case 'a':
                frequency -= 50;
                if (frequency < 50) frequency = 50;
                std::cout << "Frequency: " << frequency << " Hz\n";
                break;
            case 'p':
                dutyCycle += 5;
                if (dutyCycle > 100) dutyCycle = 100;
                std::cout << "Duty cycle: " << dutyCycle << "%\n";
                break;
            case 'l':
                dutyCycle -= 5;
                if (dutyCycle < 0) dutyCycle = 0;
                std::cout << "Duty cycle: " << dutyCycle << "%\n";
                break;
            case 'x':
                running = false;
                continue;
        }

        // Reinitialize PWM to adjust frequency and duty cycle
        softPwmStop(pwmPin);
        softPwmCreate(pwmPin, dutyCycle, 100);

        int period_us = 1000000 / frequency;
        int on_time_us = period_us * dutyCycle / 100;
        int off_time_us = period_us - on_time_us;

        // Manually handle PWM signal
        for (int i = 0; i < 10; ++i) {
            digitalWrite(pwmPin, HIGH);
            delayMicroseconds(on_time_us);
            digitalWrite(pwmPin, LOW);
            delayMicroseconds(off_time_us);
        }
    }

    softPwmStop(pwmPin);
    std::cout << "PWM stopped." << std::endl;
    return 0;
}
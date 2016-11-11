#include <iostream>
#include <thread>
#include "Serial.h"

static void test_blocking(const std::string &serial_port)
{
    Serial serial(serial_port, true);
    
    serial.open();

    if (!serial.is_open())
        return;

    std::cout << "Connected..." << std::endl;

    std::vector<char> buf(6);
    bool f = serial.read_data(buf, 5000);
    if (f) std::cout << "ok: data readed" << std::endl;
    std::string str(buf.begin(), buf.end());
    std::cout << str << std::endl;
}

static void test_async(const std::string &serial_port)
{
    Serial serial(serial_port);
    
    serial.open();

    if (!serial.is_open())
        return;

    std::cout << "Connected..." << std::endl;
    
    serial.start_async_reception();

    serial.on_data_recv.connect(
        [](std::string data)
        {
            std::cout << data << std::endl;
        });

    serial.on_error_receiving_data.connect(
        [](std::string error)
        {
            std::cout << error << std::endl;
        });

    int times = 0;
    while (!serial.quit())
    {
        // just exec for a while and
        // force quit
        if (times == 1000)
        {
            serial.force_quit();
        }

        //Do something...
        //serial.send_data("Text\r\n");
        usleep(1000*500);
        times++;
    }
}

static void usage(char *arg)
{
    std::cerr << "Usage: " << arg << " <serial-port> <BLOCK | NOBLOCK>" << std::endl;
    std::exit(1);
}

int main(int argc, char **argv)
{
    if (argc != 3)
        usage(argv[0]);

    std::string serial_port(argv[1]);
    std::string test(argv[2]);

    if (test == "BLOCK")
        test_blocking(serial_port);
    else if(test == "NOBLOCK")
        test_async(serial_port);
    else
        usage(argv[0]);

    std::cout << "Test finished" << std::endl;

    return 0;
}

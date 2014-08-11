
#include <nanotec_driver/NanotecPort.h>
#include <cstdio>

nanotec::NanotecPort::NanotecPort() : port_()
{

}

nanotec::NanotecPort::~NanotecPort()
{
    port_.close();
}

bool nanotec::NanotecPort::openPort(std::string & port_name, int baudrate)
{
    try{ port_.open(port_name.c_str(), baudrate); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }
    return true;
}

bool nanotec::NanotecPort::sendCommand(std::string & command, std::string & reply)
{
    std::string data = "#" + command;

    printf("[DEBUG] NanotecPort Sending -> %s\n", data.c_str());
    try{ port_.write(data.c_str(), data.size()); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }

    char buffer[NANOTEC_MSG_LENGTH];
    try{ port_.readLine(buffer, NANOTEC_MSG_LENGTH, NANOTEC_TIMEOUT); }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }
    reply = buffer;
    printf("[DEBUG] NanotecPort Receiving <- %s\n", reply.c_str());

    return true;
}

// EOF

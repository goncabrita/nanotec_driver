
#include <nanotec_driver/NanotecMotor.h>
#include <boost/lexical_cast.hpp>

//! Macro for throwing an exception with a message, passing args
#define NANOTEC_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in nanotec::NanotecMotor::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

nanotec::NanotecMotor::NanotecMotor(nanotec::NanotecPort * port)
{
    port_ = port;
    id_ = 1;
}

nanotec::NanotecMotor::~NanotecMotor()
{

}

void nanotec::NanotecMotor::sender(std::string symbol)
{
    std::string command;
    std::string reply;

    command = boost::lexical_cast<std::string>(id_) + symbol + "\r";

    if( !port_->sendCommand(command, reply) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    if(command.compare(reply) != 0)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);
}

void nanotec::NanotecMotor::setter(std::string symbol, int value, int min_value, int max_value)
{
    if(value < min_value || value > max_value)
        NANOTEC_EXCEPT(nanotec::Exception, "Value is out of bounds: %d. %s (errno = %d).", value, strerror(errno), errno);

    std::string command;
    std::string reply;

    command = boost::lexical_cast<std::string>(id_) + symbol + "=" + boost::lexical_cast<std::string>(value) + "\r";

    if( !port_->sendCommand(command, reply) )
       NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    std::size_t found = reply.find_first_of("=");
    if(found == std::string::npos)
    {
        command = boost::lexical_cast<std::string>(id_) + symbol + boost::lexical_cast<std::string>(value) + "\r";
    }

    if(command.compare(reply) != 0)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);
}

void nanotec::NanotecMotor::setter(std::string symbol, int value, int * array_of_permissible_values, int num_of_permissible_values)
{
    bool got_it = false;
    for(int i=0 ; i<num_of_permissible_values ; i++)
    {
        if(value == array_of_permissible_values[i])
        {
            got_it = true;
            break;
        }
    }
    if(!got_it)
        NANOTEC_EXCEPT(nanotec::Exception, "Value is out of bounds: %d. %s (errno = %d).", value, strerror(errno), errno);

    std::string command;
    std::string reply;

    command = boost::lexical_cast<std::string>(id_) + symbol + "=" + boost::lexical_cast<std::string>(value) + "\r";

    if( !port_->sendCommand(command, reply) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    std::size_t found = reply.find_first_of("=");
    if(found == std::string::npos)
    {
        command = boost::lexical_cast<std::string>(id_) + symbol + boost::lexical_cast<std::string>(value) + "\r";
    }

    if(command.compare(reply) != 0)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);
}

int nanotec::NanotecMotor::getter(std::string symbol)
{
    std::string command;
    std::string reply;
    int value;

    command = boost::lexical_cast<std::string>(id_);
    if(symbol.length() == 1) command += "Z" + symbol + "\r";
    else command += symbol + "\r";

    if( !port_->sendCommand(command, reply) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    // Delete trailing zeroes
    if(reply.at(0) == '0')
    {
        int i=1;
        while(reply.at(i) == '0')i++;
        reply.erase(reply.begin(), reply.begin()+i);
    }

    if(symbol.compare("$") == 0) command = boost::lexical_cast<std::string>(id_) + symbol + "%d" + "\r";
    else if(symbol.length() == 1) command = boost::lexical_cast<std::string>(id_) + "Z" + symbol + "%d" + "\r";
    else command = boost::lexical_cast<std::string>(id_) + symbol + "%d\r";

    if( sscanf(reply.c_str(), command.c_str(), &value) != 1)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);

    return value;
}

void nanotec::NanotecMotor::setMotorType(int motor_type)
{
    setter(":CL_motor_type", motor_type, 0, 2);
}

int nanotec::NanotecMotor::getMotorType()
{
    return getter(":CL_motor_type");
}

void nanotec::NanotecMotor::setPhaseCurrent(int current)
{
    setter("i", current, 0, 150);
}

int nanotec::NanotecMotor::getPhaseCurrent()
{
    return getter("i");
}

void nanotec::NanotecMotor::setPhaseCurrentAtStandstill(int current)
{
    setter("r", current, 0, 150);
}

int nanotec::NanotecMotor::getPhaseCurrentAtStandstill()
{
    return getter("r");
}

void nanotec::NanotecMotor::setPeakCurrent(int current)
{
    setter(":ipeak", current, 0, 150);
}

int nanotec::NanotecMotor::getPeakCurrent()
{
    return getter(":ipeak");
}

void nanotec::NanotecMotor::setCurrentTimeConstant(int time_constant)
{
    setter(":itime", time_constant, 0, 65535);
}

int nanotec::NanotecMotor::getCurrentTimeConstant()
{
    return getter(":itime");
}

void nanotec::NanotecMotor::setStepMode(int mode)
{
    int values[] = {1, 2, 4, 5, 8, 10, 16, 32, 64, 254, 255};
    setter("g", mode, values, 11);
}

int nanotec::NanotecMotor::getStepMode()
{
    return getter("g");
}

void nanotec::NanotecMotor::setAddress(int address)
{
    setter("m", address, 1, 254);
}

int nanotec::NanotecMotor::getMotorID()
{
    return getter(":mt");
}

void nanotec::NanotecMotor::setLimitSwitchBehaviour(int internal_reference, int internal_normal, int external_reference, int external_normal)
{
    if(internal_reference != NANOTEC_REFERENCE_RUN_FORWARD && internal_reference != NANOTEC_REFERENCE_RUN_BACK)
        NANOTEC_EXCEPT(nanotec::Exception, "The limit switch beahviour for the internal reference run is not valid: 0x%X. %s (errno = %d).", internal_reference, strerror(errno), errno);

    if(internal_normal != NANOTEC_NORMAL_RUN_FORWARD && internal_normal != NANOTEC_NORMAL_RUN_BACK && internal_normal != NANOTEC_NORMAL_RUN_STOP && internal_normal != NANOTEC_NORMAL_RUN_IGNORE)
        NANOTEC_EXCEPT(nanotec::Exception, "The limit switch beahviour for the internal normal run is not valid: 0x%X. %s (errno = %d).", internal_normal, strerror(errno), errno);

    if(external_reference != NANOTEC_REFERENCE_RUN_FORWARD && external_reference != NANOTEC_REFERENCE_RUN_BACK)
        NANOTEC_EXCEPT(nanotec::Exception, "The limit switch beahviour for the external reference run is not valid: 0x%X. %s (errno = %d).", external_reference, strerror(errno), errno);

    if(external_normal != NANOTEC_NORMAL_RUN_FORWARD && external_normal != NANOTEC_NORMAL_RUN_BACK && external_normal != NANOTEC_NORMAL_RUN_STOP && external_normal != NANOTEC_NORMAL_RUN_IGNORE)
        NANOTEC_EXCEPT(nanotec::Exception, "The limit switch beahviour for the external normal run is not valid: 0x%X. %s (errno = %d).", external_normal, strerror(errno), errno);

    int buffer = 0;
    buffer += internal_reference & 0x3;
    buffer += internal_normal << 2;
    buffer += external_reference << 9;
    buffer += external_normal << 11;

    setter("l", buffer, 0, 4294967295);
}

void nanotec::NanotecMotor::getLimitSwitchBehaviour(int & internal_reference, int & internal_normal, int & external_reference, int & external_normal)
{
    int buffer = getter("l");

    internal_reference = buffer & 0x3;
    internal_normal = (buffer >> 2) & 0xF;
    external_reference = (buffer >> 9) & 0x3;
    external_normal = (buffer >> 11) & 0xF;
}

void nanotec::NanotecMotor::setErrorCorrectionMode(int mode)
{
    setter("U", mode, 0, 2);
}

int nanotec::NanotecMotor::getErrorCorrectionMode()
{
    return getter("U");
}

void nanotec::NanotecMotor::setRecordForAutoCorrection(int record)
{
    setter("F", record, 0, 32);
}

int nanotec::NanotecMotor::getRecordForAutoCorrection()
{
    return getter("F");
}

void nanotec::NanotecMotor::setEncoderDirection(int direction)
{
    setter("q", direction, 0, 1);
}

int nanotec::NanotecMotor::getEncoderDirection()
{
    return getter("q");
}

void nanotec::NanotecMotor::setSwingOutTime(int time)
{
    setter("O", time, 0, 250);
}

int nanotec::NanotecMotor::getSwingOutTime()
{
    return getter("O");
}

void nanotec::NanotecMotor::setMaximumEncoderDeviation(int deviation)
{
    setter("X", deviation, 0, 250);
}

int nanotec::NanotecMotor::getMaximumEncoderDeviation()
{
    return getter("X");
}

void nanotec::NanotecMotor::setFeedRate(int numerator, int denominator)
{
    setter(":feed_const_num", numerator, 0, 2147483647);
    setter(":feed_const_denum", denominator, 0, 2147483647);
}

void nanotec::NanotecMotor::getFeedRate(int & numerator, int & denominator)
{
    numerator = getter(":feed_const_num");
    denominator = getter("feed_const_denum");
}

void nanotec::NanotecMotor::resetPositionError(int error)
{
    if(error == 0) sender("D");
    else setter("D", error, -100000000, 100000000);
}

int nanotec::NanotecMotor::getErrorMemory(int location)
{
    if((location < 1 || location > 32) && location != -1)
        NANOTEC_EXCEPT(nanotec::Exception, "Value is out of bounds: %d. %s (errno = %d).", location, strerror(errno), errno);

    if(location == -1) location = getter("E");

    std::string command;
    std::string reply;
    int error;

    command = boost::lexical_cast<std::string>(id_) + "Z" + boost::lexical_cast<std::string>(location) + "E";

    if( !port_->sendCommand(command, reply) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    command += "%d";
    if( sscanf(reply.c_str(), command.c_str(), &error) != 1)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);

    return error;
}

int nanotec::NanotecMotor::getEncoderPosition()
{
    return getter("I");
}

int nanotec::NanotecMotor::getPosition()
{
    return getter("C");
}

bool nanotec::NanotecMotor::motorIsReferenced()
{
    return (getter(":is_referenced") == 1);
}

void nanotec::NanotecMotor::getStatus(NanotecStatus & status)
{
    int buffer = getter("$");

    status.controller_ready = ((buffer & 0x01) == 1);
    status.zero_position_reached = (((buffer >> 1) & 0x01) == 1);
    status.position_error = (((buffer >> 2) & 0x01) == 1);
    status.input_1 = (((buffer >> 3) & 0x01) == 1);
}

void nanotec::NanotecMotor::getFirmwareVersion(std::string & version)
{
    std::string command;

    command = boost::lexical_cast<std::string>(id_) + "v";

    if( !port_->sendCommand(command, version) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);
}

int nanotec::NanotecMotor::getTimeSinceFirmwareUpdate()
{
    return getter(":optime");
}

void nanotec::NanotecMotor::setDigitalInputFunction(char input, int function)
{
    if(input < 'a' || input > 'h')
        NANOTEC_EXCEPT(nanotec::Exception, "Input is out of bounds: %c. %s (errno = %d).", input, strerror(errno), errno);

    std::string command = ":port_in_" + input;
    setter(command, function, 0, 13);
}

int nanotec::NanotecMotor::getDigitalInputFunction(char input)
{
    if(input < 'a' || input > 'h')
        NANOTEC_EXCEPT(nanotec::Exception, "Input is out of bounds: %c. %s (errno = %d).", input, strerror(errno), errno);

    std::string command = ":port_in_" + input;
    return getter(command);
}

void nanotec::NanotecMotor::setDigitalOutputFunction(char output, int function)
{
    if(output < 'a' || output > 'h')
        NANOTEC_EXCEPT(nanotec::Exception, "Output is out of bounds: %c. %s (errno = %d).", output, strerror(errno), errno);

    std::string command = ":port_out_" + output;
    setter(command, function, 0, 3);
}

int nanotec::NanotecMotor::getDigitalOutputFunction(char output)
{
    if(output < 'a' || output > 'h')
        NANOTEC_EXCEPT(nanotec::Exception, "Output is out of bounds: %c. %s (errno = %d).", output, strerror(errno), errno);

    std::string command = ":port_out_" + output;
    getter(command);
}

void nanotec::NanotecMotor::setIOPolarityReversal(nanotec::NanotecIOPolarity & polarity_reversal)
{
    int buffer = 0;

    buffer += (int)(!polarity_reversal.input_1);
    buffer += ((int)(!polarity_reversal.input_2) << 1);
    buffer += ((int)(!polarity_reversal.input_3) << 2);
    buffer += ((int)(!polarity_reversal.input_4) << 3);
    buffer += ((int)(!polarity_reversal.input_5) << 4);
    buffer += ((int)(!polarity_reversal.input_6) << 5);
    buffer += ((int)(!polarity_reversal.input_7) << 7);
    buffer += ((int)(!polarity_reversal.input_8) << 8);

    buffer += ((int)(!polarity_reversal.output_1) << 16);
    buffer += ((int)(!polarity_reversal.output_2) << 17);
    buffer += ((int)(!polarity_reversal.output_3) << 18);
    buffer += ((int)(!polarity_reversal.output_4) << 19);
    buffer += ((int)(!polarity_reversal.output_5) << 20);
    buffer += ((int)(!polarity_reversal.output_6) << 21);
    buffer += ((int)(!polarity_reversal.output_7) << 22);
    buffer += ((int)(!polarity_reversal.output_8) << 23);

    buffer += ((int)(polarity_reversal.ballast_resistance) << 24);

    setter("h", buffer, 0, 4294967295);
}

void nanotec::NanotecMotor::getIOPolarityReversal(nanotec::NanotecIOPolarity & polarity_reversal)
{
    int buffer = getter("h");

    polarity_reversal.input_1 = ((buffer & 0x1) == 0);
    polarity_reversal.input_2 = (((buffer >> 1) & 0x1) == 0);
    polarity_reversal.input_3 = (((buffer >> 2) & 0x1) == 0);
    polarity_reversal.input_4 = (((buffer >> 3) & 0x1) == 0);
    polarity_reversal.input_5 = (((buffer >> 4) & 0x1) == 0);
    polarity_reversal.input_6 = (((buffer >> 5) & 0x1) == 0);
    polarity_reversal.input_7 = (((buffer >> 7) & 0x1) == 0);
    polarity_reversal.input_8 = (((buffer >> 8) & 0x1) == 0);

    polarity_reversal.output_1 = (((buffer >> 16) & 0x1) == 0);
    polarity_reversal.output_2 = (((buffer >> 17) & 0x1) == 0);
    polarity_reversal.output_3 = (((buffer >> 18) & 0x1) == 0);
    polarity_reversal.output_4 = (((buffer >> 19) & 0x1) == 0);
    polarity_reversal.output_5 = (((buffer >> 20) & 0x1) == 0);
    polarity_reversal.output_6 = (((buffer >> 21) & 0x1) == 0);
    polarity_reversal.output_7 = (((buffer >> 22) & 0x1) == 0);
    polarity_reversal.output_8 = (((buffer >> 23) & 0x1) == 0);

    polarity_reversal.ballast_resistance = (((buffer >> 24) & 0x1) == 0);
}

void nanotec::NanotecMotor::setInputDebounceTime(int time)
{
    setter("K", time, 0, 250);
}

int nanotec::NanotecMotor::getInputDebounceTime()
{
    return getter("K");
}

void nanotec::NanotecMotor::setOutputs(nanotec::NanotecOutputs & outputs)
{
    int buffer = 0;

    buffer += ((int)(outputs.output_1) << 16);
    buffer += ((int)(outputs.output_2) << 17);
    buffer += ((int)(outputs.output_3) << 18);
    buffer += ((int)(outputs.output_4) << 19);
    buffer += ((int)(outputs.output_5) << 20);
    buffer += ((int)(outputs.output_6) << 21);
    buffer += ((int)(outputs.output_7) << 22);
    buffer += ((int)(outputs.output_8) << 23);

    setter("Y", buffer, 0, 4294967295);
}

void nanotec::NanotecMotor::getInputs(nanotec::NanotecInputs & inputs)
{
    int buffer = getter("Y");

    inputs.input_1 = ((buffer & 0x1) == 0);
    inputs.input_2 = (((buffer >> 1) & 0x1) == 0);
    inputs.input_3 = (((buffer >> 2) & 0x1) == 0);
    inputs.input_4 = (((buffer >> 3) & 0x1) == 0);
    inputs.input_5 = (((buffer >> 4) & 0x1) == 0);
    inputs.input_6 = (((buffer >> 5) & 0x1) == 0);
    inputs.input_7 = (((buffer >> 7) & 0x1) == 0);
    inputs.input_8 = (((buffer >> 8) & 0x1) == 0);

    inputs.encoder_index = (((buffer >> 6) & 0x1) == 0);
}

int nanotec::NanotecMotor::readEEPROM(int address)
{
    if(address < 0 || address > 16384)
        NANOTEC_EXCEPT(nanotec::Exception, "Value is out of bounds: %d. %s (errno = %d).", address, strerror(errno), errno);

    std::string command;
    std::string reply;
    int data;

    command = boost::lexical_cast<std::string>(id_) + "(E" + boost::lexical_cast<std::string>(address);

    if( !port_->sendCommand(command, reply) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    command += "%d";
    if( sscanf(reply.c_str(), command.c_str(), &data) != 1)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);

    return data;
}

void nanotec::NanotecMotor::resetEEPROM()
{
    sender("~");
}

void nanotec::NanotecMotor::setAutomaticStatusSending(bool automatic)
{
    setter("J", (int)(automatic), 0, 1);
}

bool nanotec::NanotecMotor::getAutomaticStatusSending()
{
    return (getter("J") == 1);
}

void nanotec::NanotecMotor::startBootloader()
{
    std::string command;
    std::string reply;

    command = boost::lexical_cast<std::string>(id_) + "@S";

    if( !port_->sendCommand(command, reply) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    command = boost::lexical_cast<std::string>(id_) + "@OK";
    if(command.compare(reply) != 0)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);
}

void nanotec::NanotecMotor::setReverseClearance(int reverse_clearance)
{
    setter("z", reverse_clearance, 0, 9999);
}

int nanotec::NanotecMotor::getReverseClearance()
{
    return getter("z");
}

void nanotec::NanotecMotor::setRampType(int type)
{
    setter(":ramp_mode", type, 0, 2);
}

int nanotec::NanotecMotor::getRampType()
{
    return getter(":ramp_mode");
}

void nanotec::NanotecMotor::setWaitingTimeForSwitchingOfBrakeVoltage(int time)
{
    setter(":brake_ta", time, 0, 65535);
}

int nanotec::NanotecMotor::getWaitingTimeForSwitchingOfBrakeVoltage()
{
    return getter(":brake_ta");
}

void nanotec::NanotecMotor::setWaitingTimeForMotorMovement(int time)
{
    setter(":brake_tb", time, 0, 65535);
}

int nanotec::NanotecMotor::getWaitingTimeForMotorMovement()
{
    return getter(":brake_tb");
}

void nanotec::NanotecMotor::setWaitingTimeForSwitchingOffMotorCurrent(int time)
{
    setter(":brake_tc", time, 0, 65535);
}

int nanotec::NanotecMotor::getWaitingTimeForSwitchingOffMotorCurrent()
{
    return getter(":brake_tc");
}

void nanotec::NanotecMotor::setControllerBaudRate(int baud_rate)
{
    setter(":baud", baud_rate, 1, 12);
}

int nanotec::NanotecMotor::getControllerBaudRate()
{
    return getter(":baud");
}

void nanotec::NanotecMotor::setChecksum(bool activated)
{
    setter(":crc", (int)(activated), 0, 1);
}

bool nanotec::NanotecMotor::getChecksum()
{
    return (getter(":crc") == 1);
}

void nanotec::NanotecMotor::setHallConfiguration(int configuration)
{
    setter(":hall_mode", configuration, 0, 16777215);
}

int nanotec::NanotecMotor::getHallConfiguration()
{
    return getter(":hall_mode");
}

double nanotec::NanotecMotor::getTemperature()
{
    double x = (double)(getter(":temp_adc"));

    return (1266500.0/( 4250.0 + log10(0.33* (x/1023.0)/(1 - x/1023.0) ) * 298.0 ) - 273.0);
}

void nanotec::NanotecMotor::setQuickstopRamp(int ramp)
{
    setter(":decelquick", ramp, 0, 3000000);
}

int nanotec::NanotecMotor::getQuickstopRamp()
{
    return getter(":decelquick");
}

void nanotec::NanotecMotor::setGearFactor(int numerator, int denominator)
{
    setter(":gn", numerator, 1, 255);
    setter(":gd", denominator, 1, 255);
}

void nanotec::NanotecMotor::getGearFactor(int & numerator, int & denominator)
{
    numerator = getter(":gn");
    denominator = getter(":gd");
}

void nanotec::NanotecMotor::startMotor()
{
    sender("A");
}

void nanotec::NanotecMotor::stopMotor(bool quickstop)
{
    setter("S", (int)(quickstop), 0, 1);
}

void nanotec::NanotecMotor::loadRecordFromEEPROM(int record)
{
    setter("y", record, 1, 32);
}

void nanotec::NanotecMotor::readOutCurrentRecord(nanotec::NanotecRecord & record, int record_index)
{
    std::string command;
    std::string reply;

    command = boost::lexical_cast<std::string>(id_);
    if(record_index == 0) command += "Z|";
    else command += "Z" + boost::lexical_cast<std::string>(record_index) + "|";

    if( !port_->sendCommand(command, reply) )
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to send command: %s. %s (errno = %d).", command.c_str(), strerror(errno), errno);

    command = boost::lexical_cast<std::string>(id_);
    if(record_index == 0) command += "Z";
    else command += "Z" + boost::lexical_cast<std::string>(record_index);
    command += "p%ds%du%do%dn%db%dB%dd%dt%dW%dP%dN%d:b%d:B%d";

    if(sscanf(reply.c_str(), command.c_str(),   &record.position_mode,
                                                &record.travel_distance,
                                                &record.initial_step_frequency,
                                                &record.maximum_step_frequency,
                                                &record.second_maximum_step_frequency,
                                                &record.acceleration_ramp,
                                                &record.brake_ramp,
                                                &record.direction_of_rotation,
                                                &record.reversal_of_direction_of_rotation,
                                                &record.repetitions,
                                                &record.pause_between_repetitions,
                                                &record.continuation_record,
                                                &record.maximum_jerk_for_acceleration_ramp,
                                                &record.maximum_jerk_for_brake_ramp) != 14)
        NANOTEC_EXCEPT(nanotec::Exception, "Failed to assert reply: %s. %s (errno = %d).", reply.c_str(), strerror(errno), errno);

    record.acceleration_ramp = (int)(3000.0 / sqrt((double)(record.acceleration_ramp)) - 11.7);
    record.brake_ramp = (int)(3000.0 / sqrt((double)(record.brake_ramp)) - 11.7);
}

void nanotec::NanotecMotor::saveRecord(int record)
{
    setter(">", record, 1, 32);
}

void nanotec::NanotecMotor::setPositionMode(int mode)
{
    setter("p", mode, 1, 19);
}

int nanotec::NanotecMotor::getPositionMode()
{
    return getter("p");
}

void nanotec::NanotecMotor::setTravelDistance(int distance)
{
    setter("s", distance, -100000000, 100000000);
}

int nanotec::NanotecMotor::getTravelDistance()
{
    return getter("s");
}

void nanotec::NanotecMotor::setMinimumFrequency(int frequency)
{
    setter("u", frequency, 1, 160000);
}

int nanotec::NanotecMotor::getMinimumFrequency()
{
    return getter("u");
}

void nanotec::NanotecMotor::setMaximumFrequency(int frequency)
{
    setter("o", frequency, 1, 1000000);
}

int nanotec::NanotecMotor::getMaximumFrequency()
{
    return getter("o");
}

void nanotec::NanotecMotor::setMaximumFrequency2(int frequency)
{
    setter("n", frequency, 1, 1000000);
}

int nanotec::NanotecMotor::getMaximumFrequency2()
{
    return getter("n");
}

void nanotec::NanotecMotor::setAccelerationRamp(int ramp)
{
    setter(":accel", ramp, 1, 3000000);
}

int nanotec::NanotecMotor::getAccelerationRamp()
{
    return getter(":accel");
}

void nanotec::NanotecMotor::setBrakeRamp(int ramp)
{
    setter(":decel", ramp, 1, 3000000);
}

int nanotec::NanotecMotor::getBrakeRamp()
{
    return getter(":decel");
}

void nanotec::NanotecMotor::setDirection(int direction)
{
    setter("d", direction, 0, 1);
}

int nanotec::NanotecMotor::getDirection()
{
    return getter("d");
}

void nanotec::NanotecMotor::setChangeOfDirection(bool change)
{
    setter("t", (int)(change), 0, 1);
}

bool nanotec::NanotecMotor::getChangeOfDirection()
{
    return (getter("t") == 1);
}

void nanotec::NanotecMotor::setRepetitions(int repetitions)
{
    setter("W", repetitions, 0, 254);
}

int nanotec::NanotecMotor::getRepetitions()
{
    return getter("W");
}

void nanotec::NanotecMotor::setRecordPause(int pause)
{
    setter("P", pause, 0, 65535);
}

int nanotec::NanotecMotor::getRecordPause()
{
    return getter("P");
}

void nanotec::NanotecMotor::setContinuationRecord(int record)
{
    setter("N", record, 0, 32);
}

int nanotec::NanotecMotor::getContinuationRecord()
{
    return getter("N");
}

void nanotec::NanotecMotor::setMaximumJerk(int jerk)
{
    setter(":b", jerk, 1, 100000000);
}

int nanotec::NanotecMotor::getMaximumJerk()
{
    return getter(":b");
}

void nanotec::NanotecMotor::setMaximumBrakeJerk(int jerk)
{
    setter(":B", jerk, 1, 100000000);
}

int nanotec::NanotecMotor::getMaximumBrakeJerk()
{
    return getter(":B");
}

int nanotec::NanotecMotor::getSpeed()
{
    return getter(":v");
}

void nanotec::NanotecMotor::actuateTrigger()
{
    sender("T");
}

void nanotec::NanotecMotor::setInterpolationTimePeriod(int time)
{
    setter(":clocl_interp", time, 0, 16383);
}

int nanotec::NanotecMotor::getInterpolationTimePeriod()
{
    return getter(":clocl_interp");
}

// EOF

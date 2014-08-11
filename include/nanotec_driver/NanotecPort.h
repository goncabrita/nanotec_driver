
#include <cereal_port/CerealPort.h>

#define NANOTEC_MSG_LENGTH  256
#define NANOTEC_TIMEOUT     1000

namespace nanotec
{
    /*! \class NanotecPort NanotecPort.h "nanotec_driver/NanotecPort.h"
     *  \brief NanotecPort serial port to RS485 communication class for ROS.
     *
     * This class allows to control RS485 Nanotec motors using the protocol documented on Programming Manual V2.7.
     */
    class NanotecPort
    {
    public:
        //! Constructor
        NanotecPort();
        //! Destructor
        ~NanotecPort();

        //! Open the serial port to the RS486 network
        /*!
        *
        *  \param port_name    		Serial port name.
        *  \param baudrate  		Baudrate.
        *
        *  \return True if successful false if anything went wrong!
        */
        bool openPort(std::string & port_name, int baudrate);

        //! Send a command to the motor
        /*!
        *  The default function to send and receive data.
        *
        *  \param message    		Array of chars to send.
        *  \param message_size		Size of the message buffer.
        *  \param reply             Array of chars replied by the motor.
        *  \param reply_size		Size of the reply buffer.
        *
        *  \return True if successful false if anything went wrong!
        */
        bool sendCommand(std::string & message, std::string & reply);

    private:

        //! The serial port object
        cereal::CerealPort port_;
    };
}

// EOF

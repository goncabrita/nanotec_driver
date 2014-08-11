
#include <nanotec_driver/NanotecPort.h>

//! Types of motors
#define NANOTEC_STEPPER_MOTOR           0
#define NANOTEC_BLDC_MOTOR              1
#define NANOTEC_BLDC_MOTOR_WITH_ENCODER 2

//! Step modes
#define NANOTEC_STEP_MODE_1         1
#define NANOTEC_STEP_MODE_2         2
#define NANOTEC_STEP_MODE_4         4
#define NANOTEC_STEP_MODE_5         5
#define NANOTEC_STEP_MODE_8         8
#define NANOTEC_STEP_MODE_10        10
#define NANOTEC_STEP_MODE_16        16
#define NANOTEC_STEP_MODE_32        32
#define NANOTEC_STEP_MODE_64        64
#define NANOTEC_STEP_MODE_FEED_RATE 254
#define NANOTEC_STEP_MODE_ADAPTIVE  255

//! Limit switch behaviours
#define NANOTEC_REFERENCE_RUN_FORWARD   0b10
#define NANOTEC_REFERENCE_RUN_BACK      0b01
#define NANOTEC_NORMAL_RUN_FORWARD      0b1000
#define NANOTEC_NORMAL_RUN_BACK         0b0100
#define NANOTEC_NORMAL_RUN_STOP         0b0010
#define NANOTEC_NORMAL_RUN_IGNORE       0b0001

//! Error correction mode
#define NANOTEC_ERROR_CORRECTION_MODE_OFF           0
#define NANOTEC_ERROR_CORRECTION_MODE_AFTER_TRAVEL  1
#define NANOTEC_ERROR_CORRECTION_MODE_BEFORE_TRAVEL 2

//! Error codes for error byte in EPROM
#define NANOTEC_ERROR_LOWVOLTAGE    0x01
#define NANOTEC_ERROR_TEMP          0x02
#define NANOTEC_ERROR_TMC           0x04
#define NANOTEC_ERROR_EE            0x08
#define NANOTEC_ERROR_QEI           0x10
#define NANOTEC_ERROR_INTERVAL      0x20
#define NANOTEC_ERROR_DRIVER        0x80

//! Nanotec input functions
#define NANOTEC_INPUT_USER_DEFINED  0
#define NANOTEC_INPUT_START_RECORD  1
#define NANOTEC_INPUT_RECORD_0      2
#define NANOTEC_INPUT_RECORD_1      3
#define NANOTEC_INPUT_RECORD_2      4
#define NANOTEC_INPUT_RECORD_3      5
#define NANOTEC_INPUT_RECORD_4      6
#define NANOTEC_INPUT_SWITCH        7
#define NANOTEC_INPUT_TRIGGER       8
#define NANOTEC_INPUT_DIRECTION     9
#define NANOTEC_INPUT_ENABLE        10
#define NANOTEC_INPUT_CLOCK         11
#define NANOTEC_INPUT_CLOCK_MODE_1  12
#define NANOTEC_INPUT_CLOCK_MODE_2  13

//! Nanotec output functions
#define NANOTEC_OUTPUT_USER_DEFINED 0
#define NANOTEC_OUTPUT_READY        1
#define NANOTEC_OUTPUT_RUNNING      2
#define NANOTEC_OUTPUT_ERROR        3

//! Nanotec ramp types
#define NANOTEC_RAMP_TYPE_TRAPEZOIDAL   0
#define NANOTEC_RAMP_TYPE_SINUS         1
#define NANOTEC_RAMP_TYPE_JERK_FREE     2

//! Nanotec baud rates
#define NANOTEC_BAUDRATE_110            1
#define NANOTEC_BAUDRATE_300            2
#define NANOTEC_BAUDRATE_600            3
#define NANOTEC_BAUDRATE_1200           4
#define NANOTEC_BAUDRATE_2400           5
#define NANOTEC_BAUDRATE_4800           6
#define NANOTEC_BAUDRATE_9600           7
#define NANOTEC_BAUDRATE_14400          8
#define NANOTEC_BAUDRATE_19200          9
#define NANOTEC_BAUDRATE_38400          10
#define NANOTEC_BAUDRATE_57600          11
#define NANOTEC_BAUDRATE_115200         12

//! Nanotec position modes
#define NANOTEC_POSITION_MODE_RELATIVE                          1
#define NANOTEC_POSITION_MODE_ABSOLUTE                          2
#define NANOTEC_POSITION_MODE_INTERNAL_REF_RUN                  3
#define NANOTEC_POSITION_MODE_EXTERNAL_REF_RUN                  4
#define NANOTEC_POSITION_MODE_SPEED                             5
#define NANOTEC_POSITION_MODE_FLAG_POSITION                     6
#define NANOTEC_POSITION_MODE_CLOCK_DIRECTION_LEFT              7
#define NANOTEC_POSITION_MODE_CLOCK_DIRECTION_RIGHT             8
#define NANOTEC_POSITION_MODE_CLOCK_DIRECTION_INTERNAL_REF_RUN  9
#define NANOTEC_POSITION_MODE_CLOCK_DIRECTION_EXTERNAL_REF_RUN  10
#define NANOTEC_POSITION_MODE_ANALOG                            11
#define NANOTEC_POSITION_MODE_JOYSTICK                          12
#define NANOTEC_POSITION_MODE_ANALOG_POSITION                   13
#define NANOTEC_POSITION_MODE_HW_REFERENCE                      14
#define NANOTEC_POSITION_MODE_TORQUE                            15
#define NANOTEC_POSITION_MODE_CL_QUICK_TEST                     16
#define NANOTEC_POSITION_MODE_CL_TEST                           17
#define NANOTEC_POSITION_MODE_CL_AUTOTUNE                       18
#define NANOTEC_POSITION_MODE_CL_QUICK_TEST_2                   19

//! Direction of rotation
#define NANOTEC_DIRECTION_LEFT    0
#define NANOTEC_DIRECTION_RIGHT   1

namespace nanotec
{
    //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
    #define DEF_NANOTEC_EXCEPTION(name, parent) \
    class name : public parent { \
        public: \
        name(const char* msg) : parent(msg) {} \
    }

    //! A standard exception
    DEF_NANOTEC_EXCEPTION(Exception, std::runtime_error);

    #undef DEF_NANOTEC_EXCEPTION

    typedef struct _NanotecStatus
    {
        bool controller_ready;
        bool zero_position_reached;
        bool position_error;
        bool input_1;

    } NanotecStatus;

    typedef struct _NanotecIOPolarity
    {
        bool input_1;
        bool input_2;
        bool input_3;
        bool input_4;
        bool input_5;
        bool input_6;
        bool input_7;
        bool input_8;

        bool output_1;
        bool output_2;
        bool output_3;
        bool output_4;
        bool output_5;
        bool output_6;
        bool output_7;
        bool output_8;

        bool ballast_resistance;

    } NanotecIOPolarity;

    typedef struct _NanotecOutputs
    {
        bool output_1;
        bool output_2;
        bool output_3;
        bool output_4;
        bool output_5;
        bool output_6;
        bool output_7;
        bool output_8;

    } NanotecOutputs;

    typedef struct _NanotecInputs
    {
        bool input_1;
        bool input_2;
        bool input_3;
        bool input_4;
        bool input_5;
        bool input_6;
        bool input_7;
        bool input_8;

        bool encoder_index;

    } NanotecInputs;

    typedef struct _NanotecRecord
    {
        int position_mode;
        int travel_distance;
        int initial_step_frequency;
        int maximum_step_frequency;
        int second_maximum_step_frequency;
        int acceleration_ramp;
        int brake_ramp;
        int direction_of_rotation;
        int reversal_of_direction_of_rotation;
        int repetitions;
        int pause_between_repetitions;
        int continuation_record;
        int maximum_jerk_for_acceleration_ramp;
        int maximum_jerk_for_brake_ramp;

    } NanotecRecord;

    /*! \class Nanotec nanotec.h "inc/nanotec.h"
     *  \brief Nanotec motor controllers communication class for ROS.
     *
     * This class allows to control RS485 Nanotec motors using the protocol documented on Programming Manual V2.7.
     */
    class NanotecMotor
    {
    public:
        //! Constructor
        NanotecMotor(nanotec::NanotecPort * port);
        //! Destructor
        ~NanotecMotor();

        void sender(std::string symbol);

        void setter(std::string symbol, int value, int min_value, int max_value);

        void setter(std::string symbol, int value, int * array_of_permissible_values, int num_of_permissible_values);

        int getter(std::string symbol);

        //! ********************* General Commands *********************

        //! Set the type of motor
        /*!
        *  Set the type of motor, can be NANOTEC_STEPPER_MOTOR, NANOTEC_BLDC_MOTOR or NANOTEC_BLDC_MOTOR_WITH_ENCODER.
        *
        *  \param motor_type    The type of motor.
        *
        *  \sa getMotorType()
        */
        void setMotorType(int motor_type);
        //! Get the type of motor
        /*!
        *
        *  \sa setMotorType()
        *
        *  \return The type of motor, can be NANOTEC_STEPPER_MOTOR, NANOTEC_BLDC_MOTOR or NANOTEC_BLDC_MOTOR_WITH_ENCODER.
        */
        int getMotorType();

        //! Set the phase current
        /*!
        *  Sets the phase current in percent. Values above 100 should be avoided.
        *
        *  \param current    Phase current between 0 and 150.
        *
        *  \sa getPhaseCurrent()
        */
        void setPhaseCurrent(int current);
        //! Get the phase current
        /*!
        *
        *  \sa setPhaseCurrent()
        *
        *  \return The phase current between 0 and 150.
        */
        int getPhaseCurrent();

        //! Set the phase current at standstill
        /*!
        *  Sets the current of the current reduction in percent.
        *  Like the phase current, this current is relative to the end value and not relative to the phase current.
        *  Values above 100 should be avoided.
        *
        *  \param current    Phase current between 0 and 150.
        *
        *  \sa getPhaseCurrentAtStandstill()
        *  \sa setPhaseCurrent()
        */
        void setPhaseCurrentAtStandstill(int current);
        //! Get the phase current at standstill
        /*!
        *
        *  \sa setPhaseCurrentAtStandstill()
        *  \sa getPhaseCurrent()
        *
        *  \return The phase current between 0 and 150.
        */
        int getPhaseCurrentAtStandstill();

        //! Set the peak current for a BLDC motor
        /*!
        *  Sets the peak current for a BLDC motor in percent.
        *  This value must be at least as large as the set phase current otherwise the phase current is used.
        *
        *  \param current    Peak current between 0 and 150.
        *
        *  \sa getPeakCurrent()
        */
        void setPeakCurrent(int current);
        //! Get the peak current for a BLDC motor
        /*!
        *
        *  \sa setPeakCurrent()
        *
        *  \return Peak current between 0 and 150.
        */
        int getPeakCurrent();

        //! Set the current time constant for a BLDC motor
        /*!
        *  Sets the current time constant for a BLDC motor in ms.
        *  This defines the duration for which the set peak current can flow.
        *
        *  \param time_constant    Current time constant in ms between 0 and 65535.
        *
        *  \sa getCurrentTimeConstant()
        */
        void setCurrentTimeConstant(int time_constant);
        //! Get the current time constant for a BLDC motor
        /*!
        *
        *  \sa setCurrentTimeConstant()
        *
        *  \return Current time constant in ms between 0 and 65535.
        */
        int getCurrentTimeConstant();

        //! Set the step mode
        /*!
        *  Sets the step mode. The number handed over equals the number of microsteps per full step,
        *  with the exception of NANOTEC_STEP_MODE_FEED_RATE and NANOTEC_STEP_MODE_ADAPTIVE.
        *  This defines the duration for which the set peak current can flow.
        *
        *  \param mode    One of NANOTEC_STEP_MODE_* modes.
        *
        *  \sa getStepMode()
        */
        void setStepMode(int mode);
        //! Get the step mode
        /*!
        *
        *  \sa setStepMode();
        *
        *  \return Step mode, one of NANOTEC_STEP_MODE_* modes.
        */
        int getStepMode();

        //! Set the drive address
        /*!
        *  Sets the motor address. Ensure that only one controller is connected and that the newly
        *  set address is not already occupied by another motor as this would make communication impossible.
        *
        *  In addition, if rotary address switches exist on the motor controller, they must be set to 0x00 or
        *  0x80 (0 or 128) for SMCI36, SMCI47-S and PD6-N, or to 0x0 or 0x8 to PD4-N.
        *  Otherwise, the address set by the switches will be used.
        *
        *  Addresses 0 and 255 are reserved to the EEPROM.
        *
        *  \param address    A valid address, 1 to 254.
        *
        */
        void setAddress(int address);

        //! Get the motor ID
        /*!
        *  Gets the motor ID set in NanoPro.
        *
        *  This motor ID uniquely identifies the motor type, motor designation and connection type,
        *  and is used to store in the motor controller which motor is currently connected.
        *
        *  \return  The motor id.
        *
        */
        int getMotorID();

        //! Set the limit switch behaviour
        /*!
        *  Sets the limit switch behaviour.
        *
        *  Free travel means that, when the switch is reached, the controller travels away from the
        *  switch at the set low speed.
        *
        *  Stop means that, when the limit switch is reached, the controller stops immediately.
        *  The switch remains pressed.
        *
        *  \param internal_reference    Behaviour of the internal limit switch during a reference run:
        *                               Free travel forwards
        *                               Free travel backwards
        *
        *  \param internal_normal       Behaviour of the internal limit switch during a normal run:
        *                               Free travel forwards
        *                               Free travel backwards
        *                               Stop
        *                               Ignore
        *
        *  \param external_reference    Behaviour of the external limit switch during a reference run:
        *                               Free travel forwards
        *                               Free travel backwards
        *
        *  \param external_normal       Behaviour of the external limit switch during a normal run:
        *                               Free travel forwards
        *                               Free travel backwards
        *                               Stop
        *                               Ignore
        *
        *  \sa getLimitSwitchBehaviour()
        */
        void setLimitSwitchBehaviour(int internal_reference, int internal_normal, int external_reference, int external_normal);
        //! Get the limit switch behaviour
        /*!
        *
        *  \param internal_reference    Behaviour of the internal limit switch during a reference run.
        *  \param internal_normal       Behaviour of the internal limit switch during a normal run.
        *  \param external_reference    Behaviour of the external limit switch during a reference run.
        *  \param external_normal       Behaviour of the external limit switch during a normal run.
        *
        *  \sa getLimitSwitchBehaviour()
        *
        */
        void getLimitSwitchBehaviour(int & internal_reference, int & internal_normal, int & external_reference, int & external_normal);

        //! Set the error correction mode
        /*!
        *  Sets the error correction mode.
        *  In a motor without an encoder, this value must be explicitly set to 0; otherwise it will
        *  continuously attempt to make a correction because it assumes that there are step losses.
        *
        *  The correction during travel setting exsists for compatibility reasons and is equivalent
        *  To the correction after travel behaviour. To actually make a correction during travel,
        *  the closed loop mode should be used.
        *
        *  \param mode    One of NANOTEC_ERROR_CORRECTION_MODE_* modes.
        *
        *  \sa getErrorCorrectionMode()
        */
        void setErrorCorrectionMode(int mode);
        //! Get the error correction mode
        /*!
        *
        *  \sa setErrorCorrectionMode()
        *
        *  \return Error correction mode, one of NANOTEC_ERROR_CORRECTION_MODE_* modes.
        */
        int getErrorCorrectionMode();

        //! Set the record for auto correction
        /*!
        *  The ramp and the speed in the selected record (integer) are used for the correction run.
        *  If 0 is set no correction run is performed; instead, an error is output if the error correction command is active.
        *
        *  \param record    0 to 32
        *
        *  \sa getRecordForAutoCorrection()
        */
        void setRecordForAutoCorrection(int record);
        //! Get the record for the auto correction
        /*!
        *
        *  \sa setRecordForAutoCorrection()
        *
        *  \return Record for auto correction, 0 to 32
        */
        int getRecordForAutoCorrection();

        //! Set the encoder direction
        /*!
        *  If the parameter is set to 1 the encoder direction is reversed. Defaults to 0.
        *
        *  \param direction 0 (default) or 1 (reverse direction)
        *
        *  \sa getEncoderDirection()
        */
        void setEncoderDirection(int direction);
        //! Get the encoder direction
        /*!
        *
        *  \sa setEncoderDirection()
        *
        *  \return Encoder direction, 0 (default) or 1 (reverse direction)
        */
        int getEncoderDirection();

        //! Set the swing out time
        /*!
        *  Defines the settling time in 10 ms steps between the end of the run
        *  and when the position is checkedh by the encoder.
        *
        *  This parameter is only valid for the positional check after a run.
        *
        *  Between repetitions or continuation records, this position is only
        *  checked if the pause time is longer than the settling time.
        *
        *  After a record, the settling time is awaited before the motor
        *  indicates that it is ready again.
        *
        *  \param time  Swing out time in steps of 10 ms.
        *
        *  \sa getSwingOutTime()
        */
        void setSwingOutTime(int time);
        //! Get the swing out time
        /*!
        *
        *  \sa setSwingOutTime()
        *
        *  \return Swing out time in steps of 10 ms.
        */
        int getSwingOutTime();

        //! Set the maximum encoder deviation
        /*!
        *  Specifies the maximum deviation in steps between the setpoint position and the encoder position.
        *
        *  In step modes greater than 1/10 step in 1.8 degrees and 1/5 steps in 0.9 degrees motors, this
        *  value must be greater than 0 since, even then, the encoder has a lower resolution than the
        *  microsteps of the motor.
        *
        *  \param deviation Deviation in steps.
        *
        *  \sa getMaximumEncoderDeviation()
        */
        void setMaximumEncoderDeviation(int deviation);
        //! Get the maximum encoder deviation
        /*!
        *
        *  \sa setMaximumEncoderDeviation()
        *
        *  \return Deviation in steps.
        */
        int getMaximumEncoderDeviation();

        //! Set the feed rate numerator and denominator
        /*!
        *  Sets the numerator and denominator for the feed rate.
        *  This value defines the number of steps per rotation of the motor shaft for the feed rate step mode.
        *  The feed rate is only used if numerator and denominator are not equal to 0. Otherwise the encoder
        *  resolution is used.
        *
        *  \param numerator     Feed rate numerator,
        *  \param denominator   Feed rate denominator.
        *
        *  \sa getFeedRate()
        */
        void setFeedRate(int numerator, int denominator);
        //! Get the feed rate numerator and denominator
        /*!
        *
        *  \sa setFeedRate()
        *
        *  \param numerator     Feed rate numerator,
        *  \param denominator   Feed rate denominator.
        */
        void getFeedRate(int & numerator, int & denominator);

        //! Reset the position error
        /*!
        *  Resets an error in the speed monitoring and sets the current position to the position
        *  indicated by the encoder (for input without parameters C is set to I).
        *
        *  \param error     Position error, -100000000 to 100000000.
        *
        *  \sa getEncoderPosition()
        *  \sa getPosition()
        */
        void resetPositionError(int error=0);

        //! Reading out the error memory
        /*!
        *  The firmware contains 32 error memory locations. The last 32 errors are stored.
        *  When memory location is reached, the next error is again stored at memory position 1.
        *  In this case, memory position 2 contains the oldest error code that can be read out.
        *
        *  \param location  The memory location to read the error from, 1 to 32. Leave empty for last error.
        *
        *  \return Error.
        */
        int getErrorMemory(int location = -1);

        //! Get the encoder position
        /*!
        *  In motors with an encoder, this command returns the current position of the motor in
        *  motor steps as indicated by the encoder. Provided that the motor has not lost any steps,
        *  the values of getPosition() and getCurrentRecord() are the same.
        *
        *  However, it should be noted that the encoder has a resolution that is too low for step
        *  modes greater than 1/10 in 1.8 degrees and 1/5 in 0.9 degrees motors, and differences
        *  will therefore still arrise between he two values specified above.
        *
        *  \return Encoder position.
        *
        *  \sa getPosition()
        *  \sa getCurrentRecord()
        */
        int getEncoderPosition();

        //! Get the current position
        /*!
        *  Returns the current position of the motor in steps of the set step mode. This position
        *  is relative to the position of the last reference run.
        *
        *  If the motor is equipped with an angle transmitter, this value should be very close to
        *  the value of getEncoderPosition() with a very low tolerance.
        *
        *  The tolerance depends on the step mode and the motor type (0.9 or 1.8 degrees) since the
        *  angle transmitter has a lower resolution thatn the motor in microstep mode.
        *
        *  \return Current position.
        *
        *  \sa getEncoderPosition()
        */
        int getPosition();

        //! Request motor is referenced
        /*!
        *
        *  \return 1 if the motor as been referenced, 0 otherwise.
        */
        bool motorIsReferenced();

        //! Get status
        /*!
        *
        *  \return The status of the firmware.
        *
        *  \sa setAutomaticStatusSending()
        */
        void getStatus(NanotecStatus & status);

        //! Get firmware version
        /*!
        *
        *  \return The version string of the firmware.
        */
        void getFirmwareVersion(std::string & version);

        //! Get the operating time since the firmware update
        /*!
        *
        *  \return The time since the last firmware update in seconds.
        */
        int getTimeSinceFirmwareUpdate();

        //! Set a digital input function
        /*!
        *  \param input       A char from a to h.
        *  \param function    One of NANOTEC_INPUT_* functions.
        *
        *  \sa getDigitalInputFunction()
        */
        void setDigitalInputFunction(char input, int function);
        //! Get a digital input function
        /*!
        *  \param input       A char from a to h.
        *
        *  \sa setDigitalInputFunction()
        *
        *  \return Digital input function, one of NANOTEC_INPUT_* functions.
        */
        int getDigitalInputFunction(char input);

        //! Set a digital output function
        /*!
        *  \param output  A char from a to h.
        *  \param mode    One of NANOTEC_OUTPUT_* functions.
        *
        *  \sa getDigitalOutputFunction()
        */
        void setDigitalOutputFunction(char output, int function);
        //! Get a digital output function
        /*!
        *  \param output  A char from a to h.
        *
        *  \sa setDigitalOutputFunction()
        *
        *  \return Digital output function, one of NANOTEC_OUTPUT_* functions.
        */
        int getDigitalOutputFunction(char output);

        //! Set the input/output polarity reversal mask
        /*!
        *  Sets a bit mask with which the user can reverse the polarity of the inputs and outputs.
        *  If the bit of the corresponding I/O is set to 1, there is no polarity reversal.
        *  If it is set to 0 , the polarity of the I/O is reverse.
        *
        *  The bit assignment is shown below:
        *  Bit0: Input 1
        *  Bit1: Input 2
        *  Bit2: Input 3
        *  Bit3: Input 4
        *  Bit4: Input 5
        *  Bit5: Input 6
        *  Bit7: Input 7 (SMCP33 only)
        *  Bit8: Input 8 (SMCP33 only)
        *  Bit16: Output 1
        *  Bit17: Output 2
        *  Bit18: Output 3
        *  Bit19: Output 4 (SMCP33 only)
        *  Bit20: Output 5 (SMCP33 only)
        *  Bit21: Output 6 (SMCP33 only)
        *  Bit22: Output 7 (SMCP33 only)
        *  Bit23: Output 8 (SMCP33 only)
        *  Bit24: Ballast resistance
        *  All other bits are 0.
        *  If an invalid bit mask is used, it is discarded, even if the firmware confirms it correctly.
        *
        *  \param polaroty_reversal  Polarity structure.
        *
        *  \sa getIOPolarityReversal()
        */
        void setIOPolarityReversal(NanotecIOPolarity & polarity_reversal);
        //! Get the input/output polarity reversal mask
        /*!
        *  \param polaroty_reversal  Polarity structure.
        *
        *  \sa setIOPolarityReversal()
        */
        void getIOPolarityReversal(NanotecIOPolarity & polarity_reversal);

        //! Set input debounce time
        /*!
        *  Sets the time in ms during which, after a first edge on an input, there is no response to
        *  subsequent edges. There is only a response to new edges once this debounce time has elapsed
        *  (interlocking logic). Any running debounce time of an input has no influences on the detection
        *  of edges on the other inputs.
        *
        *  \param time Debounce time in ms.
        *
        *  \sa getInputDebounceTime()
        */
        void setInputDebounceTime(int time);
        //! Get input debounce time
        /*!
        *
        *  \sa setInputDebounceTime()
        *
        *  \return Debounce time in ms.
        */
        int getInputDebounceTime();

        //! Set outputs
        /*!
        *  The bit mask has 32bits.
        *  Sets the otuputs of the firmware, provided that these have been masked for free use by means of
        *  setDigitalOutputFunction().
        *
        *  Bit0: Input 1
        *  Bit1: Input 2
        *  Bit2: Input 3
        *  Bit3: Input 4
        *  Bit4: Input 5
        *  Bit5: Input 6
        *  Bit6: '0' when the encoder is at the index line, '1' otherwise
        *  Bit7: Input 7 (SMCP33 only)
        *  Bit8: Input 8 (SMCP33 only)
        *  Bit16: Output 1
        *  Bit17: Output 2
        *  Bit18: Output 3
        *  Bit19: Output 4 (SMCP33 only)
        *  Bit20: Output 5 (SMCP33 only)
        *  Bit21: Output 6 (SMCP33 only)
        *  Bit22: Output 7 (SMCP33 only)
        *  Bit23: Output 8 (SMCP33 only)
        *  All other bits are '0'
        *
        *  \param outputs  The outputs.
        *
        *  \sa getInputs()
        *  \sa setDigitalOutputFunction()
        */
        void setOutputs(NanotecOutputs & outputs);
        //! Get inputs
        /*!
        *  The bit mask has 32bits.
        *  The status of the inputs is displayed.
        *
        *  Bit0: Input 1
        *  Bit1: Input 2
        *  Bit2: Input 3
        *  Bit3: Input 4
        *  Bit4: Input 5
        *  Bit5: Input 6
        *  Bit6: '0' when the encoder is at the index line, '1' otherwise
        *  Bit7: Input 7 (SMCP33 only)
        *  Bit8: Input 8 (SMCP33 only)
        *  Bit16: Output 1
        *  Bit17: Output 2
        *  Bit18: Output 3
        *  Bit19: Output 4 (SMCP33 only)
        *  Bit20: Output 5 (SMCP33 only)
        *  Bit21: Output 6 (SMCP33 only)
        *  Bit22: Output 7 (SMCP33 only)
        *  Bit23: Output 8 (SMCP33 only)
        *  All other bits are '0'
        *
        *  \param  The inputs.
        *
        *  \sa setOutputs()
        */
        void getInputs(NanotecInputs & inputs);

        //! readEEPROM
        /*!
        *
        *  \param address  The value of the byte in the EEPROM atthe address passed in the parameter.
        *
        *  \return  The value of the EEPROM at the address provided.
        *
        *  \sa resetEEPROM()
        */
        int readEEPROM(int address);

        //! resetEEPROM
        /*!
        *  Restores the factory defaults again. The controller requires a second until new commands are accepted.
        *
        *  A motor should not be connected during a reset. After the reset, the controller should be disconnected
        *  from the power supply for a few seconds.
        *
        *  The values of :aoa and :aaa are not reset.
        *
        *  \sa readEEPROM()
        */
        void resetEEPROM();

        //! setAutomaticStatusSending
        /*!
        *  If this parameter is set to '1' the firmware independently sends the status after the end of a run.
        *
        *  \param automatic True if automatic, false otherwise.
        *
        *  \sa getAutomaticStatusSending()
        */
        void setAutomaticStatusSending(bool automatic);
        //! getAutomaticStatusSending
        /*!
        *  If this parameter is set to '1' the firmware independently sends the status after the end of a run.
        *
        *  \return True if automatic, false otherwise.
        *
        *  \sa setAutomaticStatusSending()
        */
        bool getAutomaticStatusSending();

        //! startBootloader
        /*!
        *  This command instructs the firmware to launch the bootloader.
        */
        void startBootloader();

        //! setReverseClearance
        /*!
        *  Specifies the reverse clearance in steps.
        *
        *  This setting is used to compensate for the clearance of downstream gears when there is a change in direction.
        *
        *  When there is a change in direction, the motor takes the number of steps set in the parameter before it begins
        *  incrementing the position.
        *
        *  \param reverse_clearance The reverse clearance in steps.
        *
        *  \sa getReverseClearance()
        */
        void setReverseClearance(int reverse_clearance);
        //! getReverseClearance
        /*!
        *  Gets the reverse clearance in steps.
        *
        *  This setting is used to compensate for the clearance of downstream gears when there is a change in direction.
        *
        *  When there is a change in direction, the motor takes the number of steps set in the parameter before it begins
        *  incrementing the position.
        *
        *  \return The reverse clearance in steps.
        *
        *  \sa setReverseClearance()
        */
        int getReverseClearance();

        //! setRampType
        /*!
        *  Sets the ramp in all modes.
        *  '0' = The trapezoidal ramp is selected
        *  '1' = The sinus ramp is selected
        *  '2' = The jerk-free ramp is selected
        *  This parameters applies for all modes except clock direction and torque mode
        *  (as these modes do not generally use ramp).
        *
        *  \param type  The ramp type, on of NANOTEC_RAMP_TYPE_*.
        *
        *  \sa getRampType()
        */
        void setRampType(int type);
        //! getRampType
        /*!
        *  Gets the ramp in all modes.
        *  '0' = The trapezoidal ramp is selected
        *  '1' = The sinus ramp is selected
        *  '2' = The jerk-free ramp is selected
        *  This parameters applies for all modes except clock direction and torque mode
        *  (as these modes do not generally use ramp).
        *
        *  \return  The ramp type, on of NANOTEC_RAMP_TYPE_*.
        *
        *  \sa setRampType()
        */
        int getRampType();

        //! setWaitingTimeForSwitchingOfBrakeVoltage
        /*!
        *  Sets the waiting time between switching on the motor current and switching off (trigerring)
        *  the brake in milliseconds.
        *
        *  The parameter indicates a value between 0 and 65536 milliseconds. Defaults to 0.
        *
        *  When switching on the controller, the brake becomes active first and the motor is not provided
        *  with power. First the motor current is switched on and a period of ta ms waited.
        *  Then the brake is disengaged and a period of tb is waited. Travel commands can only
        *  be executed after expiration of ta and tb.
        *
        *  \param time  Time in ms
        *
        *  \sa getWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa setWaitingTimeForMotorMovement()
        *  \sa getWaitingTimeForMotorMovement()
        *  \sa setWaitingTimeForSwitchingOffMotorCurrent()
        *  \sa getWaitingTimeForSwitchingOffMotorCurrent()
        */
        void setWaitingTimeForSwitchingOfBrakeVoltage(int time);
        //! getWaitingTimeForSwitchingOfBrakeVoltage
        /*!
        *  Gets the waiting time between switching on the motor current and switching off (trigerring)
        *  the brake in milliseconds.
        *
        *  The parameter indicates a value between 0 and 65536 milliseconds. Defaults to 0.
        *
        *  When switching on the controller, the brake becomes active first and the motor is not provided
        *  with power. First the motor current is switched on and a period of ta ms waited.
        *  Then the brake is disengaged and a period of tb is waited. Travel commands can only
        *  be executed after expiration of ta and tb.
        *
        *  \return  Time in ms
        *
        *  \sa setWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa setWaitingTimeForMotorMovement()
        *  \sa getWaitingTimeForMotorMovement()
        *  \sa setWaitingTimeForSwitchingOffMotorCurrent()
        *  \sa getWaitingTimeForSwitchingOffMotorCurrent()
        */
        int getWaitingTimeForSwitchingOfBrakeVoltage();

        //! setWaitingTimeForMotorMovement
        /*!
        *  Sets the waiting time in millisecond between switching off of the brake voltage and
        *  enabling of a motor movement.
        *
        *  \param time  Time in ms
        *
        *  \sa setWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa getWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa getWaitingTimeForMotorMovement()
        *  \sa setWaitingTimeForSwitchingOffMotorCurrent()
        *  \sa getWaitingTimeForSwitchingOffMotorCurrent()
        */
        void setWaitingTimeForMotorMovement(int time);
        //! getWaitingTimeForMotorMovement
        /*!
        *  Gets the waiting time in millisecond between switching off of the brake voltage and
        *  enabling of a motor movement.
        *
        *  \return  Time in ms
        *
        *  \sa setWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa getWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa setWaitingTimeForMotorMovement()
        *  \sa setWaitingTimeForSwitchingOffMotorCurrent()
        *  \sa getWaitingTimeForSwitchingOffMotorCurrent()
        */
        int getWaitingTimeForMotorMovement();

        //! setWaitingTimeForSwitchingOffMotorCurrent
        /*!
        *  Sets the waiting time in milliseconds between switching on of the brake voltage and
        *  switching off of the motor current.
        *
        *  \param time  Time in ms
        *
        *  \sa setWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa getWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa setWaitingTimeForMotorMovement()
        *  \sa getWaitingTimeForMotorMovement()
        *  \sa getWaitingTimeForSwitchingOffMotorCurrent()
        */
        void setWaitingTimeForSwitchingOffMotorCurrent(int time);
        //! getWaitingTimeForSwitchingOffMotorCurrent
        /*!
        *  Gets the waiting time in milliseconds between switching on of the brake voltage and
        *  switching off of the motor current.
        *
        *  \return  Time in ms
        *
        *  \sa setWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa getWaitingTimeForSwitchingOfBrakeVoltage()
        *  \sa setWaitingTimeForMotorMovement()
        *  \sa getWaitingTimeForMotorMovement()
        *  \sa setWaitingTimeForSwitchingOffMotorCurrent()
        */
        int getWaitingTimeForSwitchingOffMotorCurrent();

        //! setControllerBaudRate
        /*!
        *  Sets the baud rate of the controller. Defaults to 115200.
        *
        *  \param baud_rate     One of NANOTEC_BAUDRATE_*.
        *
        *  \sa getControllerBaudRate()
        */
        void setControllerBaudRate(int baud_rate);
        //! getControllerBaudRate
        /*!
        *  Gets the baud rate of the controller. Defaults to 115200.
        *
        *  \return One of NANOTEC_BAUDRATE_*.
        *
        *  \sa setControllerBaudRate()
        */
        int getControllerBaudRate();

        //! setChecksum
        /*!
        *  Switches on or off the check of the serial communication using a CRC checksum (cyclic redundancy check).
        *
        *  \param activated     True if the checksum is activated, false otherwise.
        *
        *  \sa getChecksum()
        */
        void setChecksum(bool activated);
        //! getChecksum
        /*!
        *  Determine if the controller is using a check on the serial communication using a CRC checksum (cyclic redundancy check).
        *
        *  \return True if the checksum is activated, false otherwise.
        *
        *  \sa setChecksum()
        */
        bool getChecksum();

        //! setHallConfiguration
        /*!
        *  The hall mode specifies the Hall Configuration of a connected brushless motor as an integer value.
        *  For example, motor types DB42S03, DB22M and DB87S01 require the value 2371605 (0x243015) and motor
        *  types DB57 and DB22L require the value 5309250 (0x510342).
        *
        *  \param configuration     The hall configuration.
        *
        *  \sa getHallConfiguration()
        */
        void setHallConfiguration(int configuration);
        //! getHallConfiguration
        /*!
        *  The hall mode specifies the Hall Configuration of a connected brushless motor as an integer value.
        *  For example, motor types DB42S03, DB22M and DB87S01 require the value 2371605 (0x243015) and motor
        *  types DB57 and DB22L require the value 5309250 (0x510342).
        *
        *  \return     The hall configuration.
        *
        *  \sa setHallConfiguration()
        */
        int getHallConfiguration();

        //! getTemperature
        /*!
        *  Returns the current temperature of the motor controller in oC.
        *
        *  \return     The temperature in oC
        *
        *  \sa setTemperature()
        */
        double getTemperature();

        //! setQuickstopRamp
        /*!
        *  Specifies the quickstop ramp in Hz/s. Travel is stopped abruptly at 0.
        *  Defaults to 3000000.
        *
        *  \param ramp     Ramp in Hz/s from 0 to 3000000.
        *
        *  \sa getQuickstopRamp()
        */
        void setQuickstopRamp(int ramp);
        //! getQuickstopRamp
        /*!
        *  Gets the quickstop ramp in Hz/s. Travel is stopped abruptly at 0.
        *  Defaults to 3000000.
        *
        *  \return Ramp in Hz/s from 0 to 3000000.
        *
        *  \sa setQuickstopRamp()
        */
        int getQuickstopRamp();

        //! Set the gear factor numerator and denominator
        /*!
        *  Used to set the gear factor.
        *
        *  \param numerator     Gear factor numerator,
        *  \param denominator   Gear factor denominator.
        *
        *  \sa getGearFactor()
        */
        void setGearFactor(int numerator, int denominator);
        //! Get the gear factor numerator and denominator
        /*!
        *
        *  \sa setGearFactor()
        *
        *  \param numerator     Gear facotr numerator,
        *  \param denominator   Gear factor denominator.
        */
        void getGearFactor(int & numerator, int & denominator);

        //! ********************* Record Commands *********************

        //! Start the motor
        /*!
        *  Starts the motor using the current parameter settings.
        *
        *  \sa stopMotor()
        */
        void startMotor();
        //! Stop the motor
        /*!
        *  \param quickstop  True if intended to stop uisng the quickstop ramp, false if intended to use the brake ramp.
        *
        *  \sa stopMotor()
        */
        void stopMotor(bool quickstop = false);

        //! Load record from EEPROM
        /*!
        *  Loads the record data of the record passed in the parameter from the EEPROM
        *
        *  \param record Record, 1 to 32.
        *
        *  \sa saveRecord()
        */
        void loadRecordFromEEPROM(int record);

        //! Read out current record
        /*!
        *  \param record        Record.
        *  \param record_index  Record index in memory, 1 to 32, currently loaded record can be accessed using 0 (default).
        */
        void readOutCurrentRecord(NanotecRecord & record, int record_index=0);

        //! Save a record to the EEPROM
        /*!
        *  This command is used to save the currently set commands (in RAM) in a record in the EEPROM.
        *  The parmeter is the record number in which the data is saved.
        *
        *  This command should not be called up during a run because the current values change during subsequent runs.
        *
        *  A record contains the following settings and commands:
        *  Position mode 'p'
        *  Travel distance 's'
        *  Initial step frequency 'u'
        *  Maximum step frequency 'o'
        *  Second maximum step frequency 'n'
        *  Acceleration ramp 'b'
        *  Brake ramp 'B'
        *  Direction of rotation 'd'
        *  Reversal of direction of rotation for repeat records 't'
        *  Repetitions 'W'
        *  Pause between repetitions and continuation records 'P'
        *  Record number of continuation record 'N'
        *  Maximum jerk for acceleration ramp ':b'
        *  Maximum jerk for brake ramp ':B'
        *
        *  \param record Record, 1 to 32.
        *
        *  \sa loadRecordFromEEPROM()
        */
        void saveRecord(int record);

        //! Set the position mode
        /*!
        *  \param mode The position mode, one of NANOTEC_POSITION_MODE_*
        *
        *  \sa getPositionMode()
        */
        void setPositionMode(int mode);
        //! Get the position mode
        /*!
        *  \return The position mode, one of NANOTEC_POSITION_MODE_*
        *
        *  \sa setPositionMode()
        */
        int getPositionMode();

        //! Set the travel distance
        /*!
        *  This command specifies the travel distance in (micro)steps. Only positive values are allowed in relative positioning.
        *  The diretion is set with setDirection().
        *
        *  For absolute positioning, this command specifices the target position. Negative values are allowed in this case.
        *  The direction of rotation set with the command setDirection() is ignored, as this results from the current position
        *  and the target position.
        *
        *  In the adaptive mode the parameter refers to full steps.
        *
        *  \param distance   The travel distance in steps (-100 000 000 to 100 000 000).
        *
        *  \sa getTravelDistance()
        *  \sa setDirection()
        *  \sa getDirection()
        */
        void setTravelDistance(int distance);
        //! SGt the travel distance
        /*!
        *  This command specifies the travel distance in (micro)steps. Only positive values are allowed in relative positioning.
        *  The diretion is set with setDirection().
        *
        *  For absolute positioning, this command specifices the target position. Negative values are allowed in this case.
        *  The direction of rotation set with the command setDirection() is ignored, as this results from the current position
        *  and the target position.
        *
        *  In the adaptive mode the parameter refers to full steps.
        *
        *  \return The travel distance in steps (-100 000 000 to 100 000 000).
        *
        *  \sa setTravelDistance()
        *  \sa setDirection()
        *  \sa getDirection()
        */
        int getTravelDistance();

        //! Set the minimum frequency
        /*!
        *  Specifies the minimum frequency in Hz (steps per second).
        *
        *  When a record starts, the motor begins rotating with the minimum speed. It then accelerates with the set ramp setAccelerationRamp()
        *  to the maximum speed setMaximumFrequency().
        *
        *  \param frequency   The minimum frequency in Hz (steps per second), 1 to 160000.
        *
        *  \sa getMinimumFrequency()
        *  \sa setMaximumFrequency()
        *  \sa getMaximumFrequency()
        *  \sa setAccelerationRamp()
        *  \sa getAccelerationRamp()
        */
        void setMinimumFrequency(int frequency);
        //! Get the minimum frequency
        /*!
        *  Specifies the minimum frequency in Hz (steps per second).
        *
        *  When a record starts, the motor begins rotating with the minimum speed. It then accelerates with the set ramp setAccelerationRamp()
        *  to the maximum speed setMaximumFrequency().
        *
        *  \return The minimum frequency in Hz (steps per second), 1 to 160000.
        *
        *  \sa setMinimumFrequency()
        *  \sa setMaximumFrequency()
        *  \sa getMaximumFrequency()
        *  \sa setAccelerationRamp()
        *  \sa getAccelerationRamp()
        */
        int getMinimumFrequency();

        //! Set the maximum frequency
        /*!
        *  Specifies the maximum frequency in Hz (steps per second).
        *
        *  When a record starts, the motor begins rotating with the minimum speed setMinimumFrequency(). It then accelerates with the set ramp setAccelerationRamp()
        *  to the maximum speed.
        *
        *  Supports higher frequencies in open loop operation:
        *  1/2 step: 32000 Hz
        *  1/4 step: 64000 Hz
        *  1/8 step: 128000 Hz
        *  1/16 step: 256000 Hz
        *  1/32 step: 512000 Hz
        *  1/64 step: 1000000 Hz
        *
        *  \param frequency   The maximum frequency in Hz (steps per second), 1 to 1000000.
        *
        *  \sa getMaximumFrequency()
        *  \sa setMinimumFrequency()
        *  \sa getMinimumFrequency()
        *  \sa setAccelerationRamp()
        *  \sa getAccelerationRamp()
        */
        void setMaximumFrequency(int frequency);
        //! Get the maximum frequency
        /*!
        *  Specifies the maximum frequency in Hz (steps per second).
        *
        *  When a record starts, the motor begins rotating with the minimum speed setMinimumFrequency(). It then accelerates with the set ramp setAccelerationRamp()
        *  to the maximum speed.
        *
        *  Supports higher frequencies in open loop operation:
        *  1/2 step: 32000 Hz
        *  1/4 step: 64000 Hz
        *  1/8 step: 128000 Hz
        *  1/16 step: 256000 Hz
        *  1/32 step: 512000 Hz
        *  1/64 step: 1000000 Hz
        *
        *  \return The maximum frequency in Hz (steps per second), 1 to 10000000.
        *
        *  \sa setMaximumFrequency()
        *  \sa setMinimumFrequency()
        *  \sa getMinimumFrequency()
        *  \sa setAccelerationRamp()
        *  \sa getAccelerationRamp()
        */
        int getMaximumFrequency();

        //! Set the maximum frequency 2
        /*!
        *  Specifies the maximum frequency 2 in Hz (steps per second).
        *
        *  This value is only applied in the flag position mode.
        *
        *  Supports higher frequencies in open loop operation:
        *  1/2 step: 32000 Hz
        *  1/4 step: 64000 Hz
        *  1/8 step: 128000 Hz
        *  1/16 step: 256000 Hz
        *  1/32 step: 512000 Hz
        *  1/64 step: 1000000 Hz
        *
        *  \param frequency   The maximum frequency in Hz (steps per second), 1 to 1000000.
        *
        *  \sa getMaximumFrequency2()
        */
        void setMaximumFrequency2(int frequency);
        //! Get the maximum frequency 2
        /*!
        *  Specifies the maximum frequency 2 in Hz (steps per second).
        *
        *  This value is only applied in the flag position mode.
        *
        *  Supports higher frequencies in open loop operation:
        *  1/2 step: 32000 Hz
        *  1/4 step: 64000 Hz
        *  1/8 step: 128000 Hz
        *  1/16 step: 256000 Hz
        *  1/32 step: 512000 Hz
        *  1/64 step: 1000000 Hz
        *
        *  \return The maximum frequency in Hz (steps per second), 1 to 10000000.
        *
        *  \sa setMaximumFrequency2()
        */
        int getMaximumFrequency2();

        //! Set the acceleration ramp
        /*!
        *  \param ramp   Acceleration ramp in Hz/s, 1 to 3000000.
        *
        *  \sa getAccelerationRamp()
        */
        void setAccelerationRamp(int ramp);
        //! Get the acceleration ramp
        /*!
        *  \return Acceleration ramp in Hz/s, 1 to 3000000.
        *
        *  \sa setAccelerationRamp()
        */
        int getAccelerationRamp();

        //! Set the brake ramp
        /*!
        *  A value of '0' means that the value set for the acceleration ramp is used for the brake ramp.
        *
        *  \param ramp   Brake ramp in Hz/s, 1 to 3000000.
        *
        *  \sa getBrakeRamp()
        */
        void setBrakeRamp(int ramp);
        //! Get the brake ramp
        /*!
        *  A value of '0' means that the value set for the acceleration ramp is used for the brake ramp.
        *
        *  \return Brake ramp in Hz/s, 1 to 3000000.
        *
        *  \sa setBrakeRamp()
        */
        int getBrakeRamp();

        //! Set the direction of rotation
        /*!
        *  For absolute positioning the command setDirection() is ignored.
        *
        *  \param direction     Direction of rotation, left or right, one of NANOTEC_DIRECTION_*
        *
        *  \sa getDirection()
        */
        void setDirection(int direction);
        //! Get the direction of rotation
        /*!
        *  For absolute positioning the command setDirection() is ignored.
        *
        *  \return Direction of rotation, left or right, one of NANOTEC_DIRECTION_*
        *
        *  \sa setDirection()
        */
        int getDirection();

        //! Set the change of rotation
        /*!
        *  In the event that this parameter is set to '1', for repeat recors the rotational direction
        *  of the motor is reversed for each repetition.
        *
        *  \param change     True if a change of direction is intended, false otherwise.
        *
        *  \sa getDirection()
        *  \sa setRepetitions()
        *  \sa getRepetitions()
        */
        void setChangeOfDirection(bool change);
        //! Get the change of rotation
        /*!
        *  In the event that this parameter is set to '1', for repeat recors the rotational direction
        *  of the motor is reversed for each repetition.
        *
        *  \return True if a change of direction is intended, false otherwise.
        *
        *  \sa setDirection()
        *  \sa setRepetitions()
        *  \sa getRepetitions()
        */
        bool getChangeOfDirection();

        //! Set the number of repetitions
        /*!
        *  Specifies the number of repetitions of the current record.
        *  A value of '0' indicates an endless number of repetitions.
        *  The default value is '1'.
        *
        *  \param repetitions    The number of repetitions, 0 to 254.
        *
        *  \sa getRepetitions()
        */
        void setRepetitions(int repetitions);
        //! Get the number of repetitions
        /*!
        *  Specifies the number of repetitions of the current record.
        *  A value of '0' indicates an endless number of repetitions.
        *  The default value is '1'.
        *
        *  \return The number of repetitions, 0 to 254.
        *
        *  \sa setRepetitions()
        */
        int getRepetitions();

        //! Set the record pause
        /*!
        *  Specifies the pause between record repetitions or between a record and a continuation
        *  record in milliseconds.
        *
        *  If a record does not have a continuation record or a repetition, the pause is not
        *  executed and the motor is ready again immediately after the end of the run.
        *
        *  \param pause The pause in milliseconds, 0 to 65535.
        *
        *  \sa getRecordPause()
        */
        void setRecordPause(int pause);
        //! Get the record pause
        /*!
        *  Specifies the pause between record repetitions or between a record and a continuation
        *  record in milliseconds.
        *
        *  If a record does not have a continuation record or a repetition, the pause is not
        *  executed and the motor is ready again immediately after the end of the run.
        *
        *  \return The pause in milliseconds, 0 to 65535.
        *
        *  \sa setRecordPause()
        */
        int getRecordPause();

        //! Set the continuation record
        /*!
        *  Specifies the number of the continuation record. If the parameter is set to '0', a
        *  continuation record is not performed.
        *
        *  \param record    Continuation record, 0 to 32.
        *
        *  \sa getContinuationRecord()
        */
        void setContinuationRecord(int record);
        //! Get the continuation record
        /*!
        *  Specifies the number of the continuation record. If the parameter is set to '0', a
        *  continuation record is not performed.
        *
        *  \return Continuation record, 0 to 32.
        *
        *  \sa setContinuationRecord()
        */
        int getContinuationRecord();

        //! Set the maximum jerk
        /*!
        *  Sets the maximum jerk for the acceleration.
        *
        *  \param jerk    Jerk, 1 to 100000000.
        *
        *  \sa getMaximumJerk()
        */
        void setMaximumJerk(int jerk);
        //! Get the maximum jerk
        /*!
        *  Gets the maximum jerk for the acceleration.
        *
        *  \return Jerk, 1 to 100000000.
        *
        *  \sa setMaximumJerk()
        */
        int getMaximumJerk();

        //! Set the maximum brake jerk
        /*!
        *  Sets the maximum jerk for the braking ramp. If the value set is '0' the same jerk is used for the
        *  both acceleration and brake ramps.
        *
        *  \param jerk    Jerk, 1 to 100000000.
        *
        *  \sa getMaximumBrakeJerk()
        *  \sa setMaximumJerk()
        */
        void setMaximumBrakeJerk(int jerk);
        //! Get the maximum brake jerk
        /*!
        *  Gets the maximum jerk for the braking ramp. If the value set is '0' the same jerk is used for the
        *  both acceleration and brake ramps.
        *
        *  \return Jerk, 1 to 100000000.
        *
        *  \sa setMaximumBrakeJerk()
        *  \sa getMaximumJerk()
        */
        int getMaximumBrakeJerk();

        //! ********************* Mode-specific commands *********************
        //! Note: Only the functions relevant to a ROS-envionment usage will be implemented

        //! Get the speed
        /*!
        *  States the current motor speed (ONLY IN SPEED MODE!!!). This will not work for motors without encoder.
        *
        *  \return Speed in steps/s.
        */
        int getSpeed();

        //! Actuate the trigger
        /*!
        *  Trigger for the flag position mode.
        *  Before triggering, the motor travels at a constant speed.
        *  After triggering, the motor finishes travelling the set distance from the position where
        *  triggering occured, and then stops.
        */
        void actuateTrigger();

        //! Set the interpolation time period for the clock direction mode
        /*!
        *  Sets the interpolation time period for the clock direction mode in 33 microsend steps.
        *
        *  \param time      Time period.
        *
        *  \sa getInterpolationTimePeriod()
        */
        void setInterpolationTimePeriod(int time);
        //! Get the interpolation time period for the clock direction mode
        /*!
        *  Gets the interpolation time period for the clock direction mode in 33 microsend steps.
        *
        *  \param time      Time period.
        *
        *  \sa setInterpolationTimePeriod()
        */
        int getInterpolationTimePeriod();

        //! ********************* Commands for JAVA program *********************
        //! Note: Not implemented.

        //! ********************* Closed loop settings *********************

        // TODO...

        //! ********************* Motor dependent load angle values determined by test runs for the closed loop mode *********************
        //! Note: Not necessary, can be accessed using NanoPro.

        //! ********************* Scope mode *********************
        //! Note: Not implemented, used by the NanoPro software.

        //! ********************* Configuration of the current controller for controllers with dspDrive *********************

        // TODO...

    private:
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

        bool echo_;

        nanotec::NanotecPort * port_;

        unsigned int id_;
    };
}

// EOF

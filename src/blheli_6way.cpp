/*
    Allows programming the parameters of individual esc's with Blhelisuite16
    FCU works in passthrough mode when enabled
*/
#include <globals.h>
#include <blheli_6way.h>
#include <OneWire.h>

OneWire esc1(MOTOR1);
OneWire esc2(MOTOR2);
OneWire esc3(MOTOR3);
OneWire esc4(MOTOR4);
OneWire esc5(MOTOR5);
OneWire esc6(MOTOR6);

enum ErrorCodes
{
    ACK_OK = 0x00,                //  Operation succeeded. No Error.
    ACK_I_UNKNOWN_ERROR = 0x01,   // Failure in the interface for unknown reason
    ACK_I_INVALID_CMD = 0x02,     // Interface recognized an unknown command
    ACK_I_INVALID_CRC = 0x03,     // Interface calculated a different CRC / data transmission form Master failed
    ACK_I_VERIFY_ERROR = 0x04,    // Interface did a successful write operation over C2, but the read back data did not match
    ACK_D_INVALID_COMMAND = 0x05, // Device communication failed and the Status was 0x00 instead of 0x0D unused
    ACK_D_COMMAND_FAILED = 0x06,  // Device communication failed and the Status was 0x02 or 0x03 instead of 0x0D unused
    ACK_D_UNKNOWN_ERROR = 0x07,   // Device communication failed and the Status was of unknow value instead of 0x0D unused
    ACK_I_INVALID_CHANNEL = 0x08, // Interface recognized: unavailable ESC Port/Pin is adressed in Multi ESC Mode
    ACK_I_INVALID_PARAM = 0x09,   // Interface recognized an invalid Parameter
    ACK_D_GENERAL_ERROR = 0x0F    // Device communication failed for unknown reason
};

byte pc_incoming[8];        // request coming from pc
byte pc_answer[9] = {0x2e}; // answer to blhelisuite request

byte esc_request[8];           // command for each esc
byte esc_incoming[9] = {0x2e}; // answer from esc to my request

byte interface_mode = 0;

int BlHeli6Way::HostInterface()
{
    if (digitalRead(BUTTON2 == LOW))
    {
        InterfaceExit();
    }
    while (digitalRead(BUTTON2) == HIGH)
    {
        if (SERIAL_BAUDRATE != 38400)
        {
            Serial.println("Serial baudrate will be changed to 38400");
            Serial.flush();
            Serial.end();
            Serial.begin(38400);
        }
        if (Serial.available() > 0)
        {
            byte packet[8];
            for (int i = 0; i < 8; i++)
            {
                packet[i] = Serial.read();
            }
            if (packet[0] == 0x2f)
            {
                for (int i = 0; i < 8; i++)
                {
                    pc_incoming[i] = packet[i];
                }
            }

            if (pc_incoming[1] == 0x30)
                InterfaceTestAlive();
            if (pc_incoming[1] == 0x31)
                ProtocolGetVersion();
            if (pc_incoming[1] == 0x32)
                InterfaceGetName();
            if (pc_incoming[1] == 0x33)
                InterfaceGetVersion();
            if (pc_incoming[1] == 0x34)
                InterfaceExit();
            if (pc_incoming[1] == 0x35)
                DeviceReset();
            if (pc_incoming[1] == 0x37)
                DeviceInitFlash();
            if (pc_incoming[1] == 0x38)
                DeviceEraseAll();
            if (pc_incoming[1] == 0x3A)
                DeviceRead();
            if (pc_incoming[1] == 0x3B)
                DeviceWrite();
            if (pc_incoming[1] == 0x3D)
                DeviceReadEEprom();
            if (pc_incoming[1] == 0x3E)
                DeviceWriteEEprom();
            if (pc_incoming[1] == 0x3F)
                InterfaceSetMode();
        }
    }
    return 1;
}

void BlHeli6Way::InterfaceErrorResponse()
{
}

void BlHeli6Way::InterfaceTestAlive() // Interface and/or device still present and responding ?
{
    static byte answer[9] = {0x2e, 0x30, 0x0, 0x0, 0x1, 0x0, ACK_OK, 0x44, 0xC2};
    for (int i = 0; i < 9; i++)
        Serial.write(answer[i]);
}

void BlHeli6Way::ProtocolGetVersion() // Retrieve Interface Name (Type) as text.
{
    static byte answer[9] = {0x2e, 0x31, 0x0, 0x0, 0x1, 0x6C, ACK_OK};
    uint16_t crc = crc_xmodem(answer, 7);

    byte crc_low = lowByte(crc);
    byte crc_high = highByte(crc);

    answer[7] = crc_high;
    answer[8] = crc_low;

    for (int i = 0; i < 9; i++)
    {
        Serial.write(answer[i]);
    }
}

void BlHeli6Way::InterfaceGetName() // Retrieve Interface version as byte value
{
    static byte answer[20] = {0x2e, 0x32, 0x0, 0x0, 0x0c, 0x6d, 0x34, 0x77, 0x41, 0x52, 0x6d, 0x31, 0x36, 0x38, 0x5f, 0x31, 0x36, ACK_OK}; // Act like arduino nano w/atmega168
    uint16_t crc = crc_xmodem(answer, 18);

    byte crc_low = lowByte(crc);
    byte crc_high = highByte(crc);

    answer[18] = crc_high;
    answer[19] = crc_low;

    for (int i = 0; i < 20; i++)
    {
        Serial.write(answer[i]);
    }
}

void BlHeli6Way::InterfaceGetVersion() // Retrieve Interface version as byte value
{
    static byte answer[10] = {0x2e, 0x33, 0x0, 0x0, 0x2, 0xC8, 0x05, ACK_OK};
    uint16_t crc = crc_xmodem(answer, 8);

    byte crc_low = lowByte(crc);
    byte crc_high = highByte(crc);

    answer[8] = crc_high;
    answer[9] = crc_low;

    for (int i = 0; i < 10; i++)
    {
        Serial.write(answer[i]);
    }
}

bool BlHeli6Way::InterfaceExit() // Exit PC Mode (SilC2: Resets the ESC's and) restarts Interface or Boxes Display Mode
{
    static byte answer[9] = {0x2e, 0x34, 0x0, 0x0, 0x1, 0x0, ACK_OK, 0x42, 0x63};
    for (int i = 0; i < 9; i++)
    {
        Serial.write(answer[i]);
    }

    if (SERIAL_BAUDRATE != 38400)
    {
        Serial.end();
        Serial.begin(SERIAL_BAUDRATE);
    }

    return true;
}

void BlHeli6Way::DeviceReset() // Reset connected Target (ESC)
{
}

void BlHeli6Way::DeviceInitFlash() // Enable Flash access to Target MCU and retrieve MCU info
{
    byte esc_nr = pc_incoming[5];
    static byte answer[12] = {0x2e, 0x37, 0x0, 0x0, 0x03, 0, 0, 0, interface_mode, ACK_OK};
    uint16_t crc = crc_xmodem(answer, 10);

    byte crc_low = lowByte(crc);
    byte crc_high = highByte(crc);

    answer[10] = crc_high;
    answer[11] = crc_low;

    for (int i = 0; i < 12; i++)
    {
        Serial.write(answer[i]);
    }
}

void BlHeli6Way::DeviceEraseAll() // Erase whole memory of Target MCU, valid for SilC2, AtmSK not SilBLB not AtmBLB
{
}

void BlHeli6Way::DeviceRead() // Read memory of Target MCU
{
}

void BlHeli6Way::DeviceWrite() // Write to memory of Target MCU
{
}

void BlHeli6Way::DeviceReadEEprom() // Read EEprom of Target Atmel MCU
{
}

void BlHeli6Way::DeviceWriteEEprom() // Write to EEprom of Target Atmel MCU
{
}

void BlHeli6Way::InterfaceSetMode() // Set interface mode
{
    interface_mode = pc_incoming[5];
    static byte answer[9] = {0x2e, 0x3F, 0x0, 0x0, 0x1, interface_mode, ACK_OK};
    uint16_t crc = crc_xmodem(answer, 7);

    byte crc_low = lowByte(crc);
    byte crc_high = highByte(crc);

    answer[7] = crc_high;
    answer[8] = crc_low;

    for (int i = 0; i < 9; i++)
    {
        Serial.write(answer[i]);
    }
}
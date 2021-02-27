#ifndef BLHELI_6WAY_H_
#define BLHELI_6WAY_H_

class BlHeli6Way
{
private:
    void InterfaceErrorResponse();
    void InterfaceTestAlive();
    void ProtocolGetVersion();
    void InterfaceGetName();
    void InterfaceGetVersion();
    void DeviceReset();
    void DeviceInitFlash();
    void DeviceEraseAll();
    void DeviceRead();
    void DeviceWrite();
    void DeviceReadEEprom();
    void DeviceWriteEEprom();
    void InterfaceSetMode();

public:
    int HostInterface();
    bool InterfaceExit();
};

#endif
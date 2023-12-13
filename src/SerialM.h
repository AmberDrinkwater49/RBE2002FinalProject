#ifndef SERIALM
#define SERIALM

#include <Romi32U4.h>

class SerialM{
    private:
        String serString1 = "";
    public:
        void sendMessage(const String & topic, const String & message);
        bool checkSerial1(void);
        String receivedString(void);
};

#endif
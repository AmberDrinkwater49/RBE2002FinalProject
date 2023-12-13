#include <Romi32U4.h>
#include <SerialM.h>

    void SerialM::sendMessage(const String & topic, const String & message)
    {
        Serial1.println(topic + String(':') + message);
    }
    
    bool SerialM::checkSerial1(void)
    {
        while(Serial1.available())
        {
            char c = Serial1.read();
            serString1 += c;

            if(c == '\n')
            {
                return true;
            }
        }

        return false;
    }

    String SerialM::receivedString(void)
    {
        if(checkSerial1())
        {
            Serial.print("Rec'd:\t");
            Serial.print(serString1);
            serString1 = "";
        }
        else{
            return "";
        }
    }
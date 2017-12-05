#ifndef McbRosConfiguration_h
#define McbRosConfiguration_h

#include <IPAddress.h>
#include <EEPROM.h>
#include <stdint.h>

class McbRosConfiguration {
public:
    McbRosConfiguration(void) {
        Serial.begin(115200);
        Serial.setTimeout(100);
    }

    void run(void) { // !! NOTE: THIS IS A BLOCKING FUNCTION !!
        bool exit = false;

        Serial.println(F("\nROS Configuration"));
        printMenu();
        printWaitCommand();

        while (!exit) {
            // check for serial commands
            if (Serial.available() > 0) {
                char cmd = Serial.read();

                switch (cmd) {
                case 'c':
                case 'C':
                    if (mcbRosSettings_.saveFlag == blank) {
                        Serial.println(F("\nError: Must load or modify settings first"));
                    }
                    else {
                        printSettings();
                    }
                    printWaitCommand();
                    break;

                case 'e':
                case 'E':
                    // read current settings from EEPROM
                    EEPROM.get(eepromAddr_, mcbRosSettings_);

                    if (mcbRosSettings_.saveFlag == eeprom) {
                        ipSet = true;
                        macSet = true;
                        rosNamespaceSet = true;
                        Serial.println(F("\n Currently saved settings:"));
                        printSettings();
                    }
                    else {
                        mcbRosSettings_.saveFlag = blank;
                        Serial.println(F("No saved settings found"));
                    }
                    printWaitCommand();
                    break;

                case 'n':
                case 'N': // sets namespace to use
                {
                    Serial.println(F("\nEnter desired namespace:"));
                    while (!Serial.available()); // wait for user input
                    String desiredName = Serial.readString();

                    // check that the desired name is <25 characters
                    if (desiredName.length() > 25) {
                        Serial.println(F("\nInvalid Name: must be less than 25 characters"));
                    }
                    else {
                        rosNamespaceSet = true;
                        mcbRosSettings_.saveFlag = unsaved;
                        char zeros[25] = { 0 };
                        memcpy(mcbRosSettings_.rosNamespace, zeros, 25); // reset name to zeros
                        memcpy(mcbRosSettings_.rosNamespace, desiredName.c_str(), desiredName.length());
                        Serial.print("namespace = ");
                        Serial.println(mcbRosSettings_.rosNamespace);
                    }
                    printWaitCommand();
                    break;
                }
                case 'i':
                case 'I':
                {
                    Serial.println(F("\nEnter last octet (192.168.0.X) of desired IP address:"));
                    while (!Serial.available()); // wait for user input
                    String ipString = Serial.readString();
                    long ipLong = ipString.toInt();
                    if ((ipLong>255) | (ipLong<2)) {
                        Serial.println(F("Invalid input: must be integer between 2-255"));
                    }
                    else {
                        ipSet = true;
                        mcbRosSettings_.saveFlag = unsaved;
                        mcbRosSettings_.ip = static_cast<uint8_t>(ipLong);
                        Serial.print(F("IP set to: 192.168.0."));
                        Serial.println(mcbRosSettings_.ip);
                    }
                    printWaitCommand();
                    break;
                }
                case 'm':
                case 'M':
                {
                    Serial.println(F("\nEnter last octet (DE:AD:BE:EF:FE:XX) of desired MAC address:"));
                    while (!Serial.available()); // wait for user input
                    String macString = Serial.readString();
                    long macLong = strtoul(macString.c_str(), NULL, 16); // interpret as HEX
                    if ((macLong > 255) | (macLong<1)) {
                        Serial.println(F("Invalid input: must be hex value between 01-FF"));
                    }
                    else {
                        macSet = true;
                        mcbRosSettings_.saveFlag = unsaved;
                        mcbRosSettings_.mac = static_cast<uint8_t>(macLong);
                        Serial.print(F("MAC = DE:AD:BE:EF:FE:"));
                        Serial.println(mcbRosSettings_.mac, HEX);
                    }
                    printWaitCommand();
                    break;
                }

                case 's':
                case 'S':
                    if (mcbRosSettings_.saveFlag == unsaved && rosNamespaceSet && ipSet && macSet) {
                        // save current settings to EEPROM
                        mcbRosSettings_.saveFlag = eeprom;
                        EEPROM.put(eepromAddr_, mcbRosSettings_);
                        Serial.println(F("\nSaved settings to EEPROM\n"));
                        printSettings();
                        exit = true;
                    }
                    else if (mcbRosSettings_.saveFlag == eeprom) {
                        Serial.println(F("\nNo settings changed, using previously saved values from EEPROM"));
                        printSettings();
                        exit = true;
                    }
                    else {
                        Serial.println(F("\nError: Check that all parameters have been set first"));
                        printWaitCommand();
                    }
                    break;

                case 'x':
                case 'X':
                    Serial.println(F("\nExiting without saving to EEPROM"));
                    Serial.println(F("Changes will only apply for this session"));
                    printSettings();
                    exit = true;
                    break;

                default:
                    Serial.println(F("Command not recognized"));
                    printMenu();
                    printWaitCommand();
                }
            }
        }
    }

    bool runOnce(void) { // Non-Blocking; returns false when user selects 's' or 'x'
        bool keepRunning = true;

        // check for serial commands
        if (Serial.available() > 0) {
            char cmd = Serial.read();

            switch (cmd) {
            case 'c':
            case 'C':
                if (mcbRosSettings_.saveFlag == blank) {
                    Serial.println(F("\nError: Must load or modify settings first"));
                }
                else {
                    printSettings();
                }
                printWaitCommand();
                break;

            case 'e':
            case 'E':
                // read current settings from EEPROM
                EEPROM.get(eepromAddr_, mcbRosSettings_);

                if (mcbRosSettings_.saveFlag == eeprom) {
                    ipSet = true;
                    macSet = true;
                    rosNamespaceSet = true;
                    Serial.println(F("\n Currently saved settings:"));
                    printSettings();
                }
                else {
                    mcbRosSettings_.saveFlag = blank;
                    Serial.println(F("No saved settings found"));
                }
                printWaitCommand();
                break;

            case 'n':
            case 'N': // sets namespace to use
            {
                Serial.println(F("\nEnter desired namespace:"));
                while (!Serial.available()); // wait for user input
                String desiredName = Serial.readString();

                // check that the desired name is <25 characters
                if (desiredName.length() > 25) {
                    Serial.println(F("\nInvalid Name: must be less than 25 characters"));
                }
                else {
                    rosNamespaceSet = true;
                    mcbRosSettings_.saveFlag = unsaved;
                    char zeros[25] = { 0 };
                    memcpy(mcbRosSettings_.rosNamespace, zeros, 25); // reset name to zeros
                    memcpy(mcbRosSettings_.rosNamespace, desiredName.c_str(), desiredName.length());
                    Serial.print("namespace = ");
                    Serial.println(mcbRosSettings_.rosNamespace);
                }
                printWaitCommand();
                break;
            }
            case 'i':
            case 'I':
            {
                Serial.println(F("\nEnter last octet (192.168.0.X) of desired IP address:"));
                while (!Serial.available()); // wait for user input
                String ipString = Serial.readString();
                long ipLong = ipString.toInt();
                if ((ipLong>255) | (ipLong<2)) {
                    Serial.println(F("Invalid input: must be integer between 2-255"));
                }
                else {
                    ipSet = true;
                    mcbRosSettings_.saveFlag = unsaved;
                    mcbRosSettings_.ip = static_cast<uint8_t>(ipLong);
                    Serial.print(F("IP set to: 192.168.0."));
                    Serial.println(mcbRosSettings_.ip);
                }
                printWaitCommand();
                break;
            }
            case 'm':
            case 'M':
            {
                Serial.println(F("\nEnter last octet (DE:AD:BE:EF:FE:XX) of desired MAC address:"));
                while (!Serial.available()); // wait for user input
                String macString = Serial.readString();
                long macLong = strtoul(macString.c_str(), NULL, 16); // interpret as HEX
                if ((macLong > 255) | (macLong<1)) {
                    Serial.println(F("Invalid input: must be hex value between 01-FF"));
                }
                else {
                    macSet = true;
                    mcbRosSettings_.saveFlag = unsaved;
                    mcbRosSettings_.mac = static_cast<uint8_t>(macLong);
                    Serial.print(F("MAC = DE:AD:BE:EF:FE:"));
                    Serial.println(mcbRosSettings_.mac, HEX);
                }
                printWaitCommand();
                break;
            }

            case 's':
            case 'S':
                if (mcbRosSettings_.saveFlag == unsaved && rosNamespaceSet && ipSet && macSet) {
                    // save current settings to EEPROM
                    mcbRosSettings_.saveFlag = eeprom;
                    EEPROM.put(eepromAddr_, mcbRosSettings_);
                    Serial.println(F("\nSaved settings to EEPROM\n"));
                    printSettings();
                    keepRunning = false;
                }
                else if (mcbRosSettings_.saveFlag == eeprom) {
                    Serial.println(F("\nNo settings changed, using previously saved values from EEPROM"));
                    printSettings();
                    keepRunning = false;
                }
                else {
                    Serial.println(F("\nError: Check that all parameters have been set first"));
                    printWaitCommand();
                }
                break;

            case 'x':
            case 'X':
                Serial.println(F("\nExiting without saving to EEPROM"));
                Serial.println(F("Any changes will only apply to this current session"));
                printSettings();
                keepRunning = false;
                return keepRunning;

            default:
                Serial.println(F("Command not recognized"));
                printMenu();
                printWaitCommand();
            }
        }
        return keepRunning;
    }

    void getSettingsFromEeprom(void) {
        // read current settings from EEPROM
        EEPROM.get(eepromAddr_, mcbRosSettings_);
    }

    String getNamespace(void) {
        return String(mcbRosSettings_.rosNamespace);
    }

    uint8_t getIP(void) {
        return mcbRosSettings_.ip;
    }

    uint8_t getMac(void) {
        return mcbRosSettings_.mac;
    }

    bool wasSaved(void) {
        return (mcbRosSettings_.saveFlag == eeprom);
    }

    void printMenu(void) {
        Serial.println(F("-------------------"));
        Serial.println(F("Commands:"));
        Serial.println(F("'c' -> view current settings"));
        Serial.println(F("'e' -> load saved settings from EEPROM (overwrites current values)"));
        Serial.println(F("'i' -> set IP address"));
        Serial.println(F("'m' -> set MAC address"));
        Serial.println(F("'n' -> set namespace"));
        Serial.println(F("'s' -> save settings to EEPROM and exit"));
        Serial.println(F("'x' -> exit without saving"));
        Serial.println(F("-------------------\n"));
    }

    void printSettings(void) {
        Serial.print(F("namespace = "));
        Serial.println(mcbRosSettings_.rosNamespace);
        Serial.print(F("ipSaved = 192.168.0."));
        Serial.println(mcbRosSettings_.ip);
        Serial.print(F("macSaved = DE:AD:BE:EF:FE:"));
        Serial.println(mcbRosSettings_.mac, HEX);
        Serial.println(F("\n"));
    }

    void printWaitCommand(void) {
        Serial.println(F("Waiting for command..."));
    }

private:
    enum SaveState {blank, eeprom, unsaved};
    struct {
        uint8_t ip;  // last byte of Wiznet IP
        uint8_t mac; // last byte of Wiznet MAC
        char rosNamespace[25];
        SaveState saveFlag = blank; // equals saveFlagKey_ if settings have been previously saved
    } mcbRosSettings_;

    uint16_t eepromAddr_ = 93; // start address for EEPROM data

    bool ipSet = false;
    bool macSet = false;
    bool rosNamespaceSet = false;
};

#endif
#include "MIDIUSB.h"
// script settings
#define CHANNEL 0
#define DEBUG 1
#define DEBOUNCE 300
// IO
#define SWITCH1 A0
#define SWITCH2 A1
#define BANKSWITCH A2

bool debug = DEBUG;

class Footswitch {
  public:
    Footswitch(uint8_t p) : pin(p) {}
    
    // setup pin 
    void setup() {
      pinMode(pin, INPUT_PULLUP);
    }

    // looking for changes in the state of the footswitch
    void update(byte control) {
      press(digitalRead(pin), control);
    }

    // returns state of the footswitch
    bool isPressed() {
      return pressed;
    }
    
  private:
    const uint8_t pin;
    const uint8_t channel = CHANNEL;
    const long debounceTime = DEBOUNCE;
    unsigned long lastPressed;
    bool pressed = 0;
    
    // sends midi signal on state change
    void press(boolean state, byte control) {
      if (state == pressed || (millis() - lastPressed  <= debounceTime)) {
        return;
      }
      state ? controlChange(channel, control, 127) : controlChange(channel, control, 0);
      lastPressed = millis();
      pressed = state;
      if (debug) {
        Serial.println();
        Serial.print("control: ");
        Serial.print(control);
        Serial.println();
        Serial.print("state: ");
        Serial.print(state);
        Serial.println();
      }
    }

    // 
    void controlChange(byte channel, byte control, byte value) {
      midiEventPacket_t event = {0x0B, 0xB0 | channel, control, value};
      MidiUSB.sendMIDI(event);
      MidiUSB.flush();
    }
};

Footswitch footswitchs[] = {
  SWITCH1,
  SWITCH2,
};

const uint8_t NumSwitches = sizeof(footswitchs) / sizeof(Footswitch);

class BankSwitch {
  public:
    // setup pin
    void setup() {
      pinMode(pin, INPUT_PULLUP);
    }

    //  looking for state change
    void update() {
      flip(!digitalRead(pin));
    }

    // returns the current bank. Incriments by the number of footswitches
    byte bankNum() {
      return bank;
    }
    
  private:
    const uint8_t pin = BANKSWITCH;
    const long debounceTime = DEBOUNCE;
    unsigned long lastFlipped;
    bool flipped = 0;
    byte bank = 0; 

    // changes "bank" on state change
    void flip(boolean state) {
      // stops the changing of banks while an fx loop is enabled
      // will run until loop is disabled. then will change bank as normal
      for (byte i = 0; i < NumSwitches; i++) {
        if (footswitchs[i].isPressed()) return;
      }
      if (state == flipped || (millis() - lastFlipped  <= debounceTime)) {
        return;
      }
      bank = state ? bank + NumSwitches : bank - NumSwitches;
      lastFlipped = millis();
      flipped = state;
      if (debug) {
        Serial.println();
        Serial.print("bank: ");
        Serial.print(bank);
        Serial.println();
        Serial.print("state: ");
        Serial.print(state);
        Serial.println();
      }
    }
};

BankSwitch bankswitch;

void setup() {
  if (debug) {
    Serial.begin(9600);
  }
  for (byte i = 0; i < NumSwitches; i++) {
    footswitchs[i].setup();
  }
  bankswitch.setup();
}

void loop() {
  for (byte i = 0; i < NumSwitches; i++) {
    footswitchs[i].update(i + bankswitch.bankNum());
  }
  bankswitch.update();
}

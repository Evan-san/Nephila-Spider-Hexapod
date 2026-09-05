const int deadzone = 35;       // ignore small stick drift
bool ps3Connected = false;
char mode = 'x'; // current walking mode, controlled over Serial or PS3
void onPs3Connect() {
  ps3Connected = true;
  Serial.println("PS3 controller connected.");
}

void onPs3Disconnect() {
  ps3Connected = false;
  mode = 'x';
  Serial.println("PS3 controller disconnected. Robot stopped.");
}

// Reads controller state and sets `mode` just like a Serial command would
void updateModeFromPs3() {
  if (!Ps3.isConnected()) return;

  int ly = Ps3.data.analog.stick.ly; // left stick vertical, -128..127
  int lx = Ps3.data.analog.stick.lx; // left stick horizontal, -128..127

  // Triangle = stand neutral / stop
  if (Ps3.event.button_down.triangle) {
    mode = 'n';
    return;
  }

 

  if (ly < -deadzone) {
    mode = 'w';        // stick pushed up -> forward
  } else if (ly > deadzone) {
    mode = 's';        // stick pulled down -> backward
  } else if (lx > deadzone) {
    mode = 'd';        // stick right -> turn right
  } else if (lx < -deadzone) {
    mode = 'a';        // stick left -> turn left
  } else {
    mode = 'x';        // stick centered -> hold position
  }
}

void printEsp32Mac() {
  // Helpful once, to know which MAC to pair the PS3 controller to
  Serial.print("ESP32 Bluetooth MAC: ");
  Serial.println(Ps3.getAddress());
}


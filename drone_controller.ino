#define HEADER1   0xAB
#define HEADER2   0xCD

#define LSTICK_X  0x01
#define LSTICK_Y  0x02
#define RSTICK_X  0x03
#define RSTICK_Y  0x04
#define TRIM      0x05
#define TAKEOFF   0x06
#define COMPASS   0x07
#define SPEED     0x08

#define LSTICK_X_PIN  6
#define LSTICK_Y_PIN  5
#define RSTICK_X_PIN  9
#define RSTICK_Y_PIN  10
#define TRIM_PIN      15  // A1
#define TAKEOFF_PIN   16  // A2
#define COMPASS_PIN   17  // A3
#define SPEED_PIN     18  // A4

unsigned char header1;
unsigned char header2;
unsigned char cmd;
unsigned char val;

int ledPin = 13;

void setup() {
  set_button(TRIM_PIN, 0);
  set_button(TAKEOFF_PIN, 0);
  set_button(COMPASS_PIN, 0);
  set_button(SPEED_PIN, 0);
  set_joystick(LSTICK_X_PIN, 128);
  set_joystick(LSTICK_Y_PIN, 128);
  set_joystick(RSTICK_X_PIN, 128);
  set_joystick(RSTICK_Y_PIN, 128);
  // start serial port at 115200 baud:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void set_button(int pin, int val) {
  if (val) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  } else {
    pinMode(pin, INPUT);
  }
}

int scale_joystick_value(int val) {
  float m0_min = 0;
  float m0_max = 3.3;
  float drone_min = 0.4;
  float drone_max = 2.6;
  int output_min = round(255.0*(drone_min/m0_max));
  int output_max = round(255.0*(drone_max/m0_max));
  
  return round(constrain(map(val, 0, 255, output_min, output_max), output_min, output_max));
  //return output_min;
}

void set_joystick(int pin, int val) {
  analogWrite(pin, scale_joystick_value(val));
}

void loop() {
  while(!(Serial.available()));
  header1 = Serial.read();
  if (header1 == HEADER1) {
    while(!(Serial.available()));
    header2 = Serial.read();
    if (header2 == HEADER2) {
      while(!(Serial.available()));
      cmd = Serial.read();
      while(!(Serial.available()));
      val = Serial.read();
      switch (cmd) {
        case LSTICK_X:
          Serial.print("LSTICK_X\n");
          set_joystick(LSTICK_X_PIN, val);
          break;
        case LSTICK_Y:
          Serial.print("LSTICK_Y\n");
          set_joystick(LSTICK_Y_PIN, val);
          break;
        case RSTICK_X:
          Serial.print("RSTICK_X\n");
          set_joystick(RSTICK_X_PIN, val);
          break;
        case RSTICK_Y:
          Serial.print("RSTICK_Y\n");
          set_joystick(RSTICK_Y_PIN, val);
          break;
        case TRIM:
          Serial.print("TRIM\n");
          set_button(TRIM_PIN, val);
          break;
        case TAKEOFF:
          Serial.print("TAKEOFF\n");
          set_button(TAKEOFF_PIN, val);
          break;
        case COMPASS:
          Serial.print("COMPASS\n");
          set_button(COMPASS_PIN, val);
          break;
        case SPEED:
          Serial.print("SPEED\n");
          set_button(SPEED_PIN, val);
          break;
      }
    }
  }
}

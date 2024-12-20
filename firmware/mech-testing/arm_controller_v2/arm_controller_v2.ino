/* BASE CONTROL BUTTONS */
#define BASE_ROLL_TOP 22
#define BASE_ROLL_BOTTOM 24
#define BASE_PITCH_TOP 27
#define BASE_PITCH_BOTTOM 29
/* BASE DRIVER WIRES */
#define BASE_ROLL_STEP 13
#define BASE_ROLL_DIR 12
#define BASE_PITCH_STEP 11
#define BASE_PITCH_DIR 10

/* ELBOW CONTROL BUTTONS */
#define ELBOW_TOP 30
#define ELBOW_BOTTOM 32
/* ELBOW DRIVER WIRES */
#define ELBOW_STEP 9
#define ELBOW_DIR 8

/* WRIST CONTROL WIRES */
#define WRIST_ROLL_TOP 35
#define WRIST_ROLL_BOTTOM 37
#define WRIST_PITCH_TOP 38
#define WRIST_PITCH_BOTTOM 40
/* WRIST DRIVER WIRES */
#define WRIST_ROLL_STEP 7
#define WRIST_ROLL_DIR 6
#define WRIST_PITCH_STEP 5
#define WRIST_PITCH_DIR 4

/* CLAW CONTROL WIRES */
#define CLAW_TOP 43
#define CLAW_BOTTOM 45
/* CLAW DRIVER WIRES */
#define CLAW_STEP 3
#define CLAW_DIR 2

/* ADJUST TO CHANGE SPEED (LOWER = FASTER) */
#define LOOP_DELAY 50000

void setup()
{
  // INPUT
  pinMode(BASE_ROLL_TOP, INPUT_PULLUP);
  pinMode(BASE_ROLL_BOTTOM, INPUT_PULLUP);

  pinMode(BASE_PITCH_TOP, INPUT_PULLUP);
  pinMode(BASE_PITCH_BOTTOM, INPUT_PULLUP);

  pinMode(ELBOW_TOP, INPUT_PULLUP);
  pinMode(ELBOW_BOTTOM, INPUT_PULLUP);

  pinMode(WRIST_ROLL_TOP, INPUT_PULLUP);
  pinMode(WRIST_ROLL_BOTTOM, INPUT_PULLUP);

  pinMode(WRIST_PITCH_TOP, INPUT_PULLUP);
  pinMode(WRIST_PITCH_BOTTOM, INPUT_PULLUP);

  pinMode(CLAW_TOP, INPUT_PULLUP);
  pinMode(CLAW_BOTTOM, INPUT_PULLUP);

  // OUTPUT
  pinMode(BASE_ROLL_STEP, OUTPUT);
  pinMode(BASE_ROLL_DIR, OUTPUT);

  pinMode(BASE_PITCH_STEP, OUTPUT);
  pinMode(BASE_PITCH_DIR, OUTPUT);

  pinMode(ELBOW_STEP, OUTPUT);
  pinMode(ELBOW_DIR, OUTPUT);

  pinMode(WRIST_ROLL_STEP, OUTPUT);
  pinMode(WRIST_ROLL_DIR, OUTPUT);

  pinMode(WRIST_PITCH_STEP, OUTPUT);
  pinMode(WRIST_PITCH_DIR, OUTPUT);

  pinMode(CLAW_STEP, OUTPUT);
  pinMode(CLAW_DIR, OUTPUT);
}

void handleButtonSingleMotor(uint8_t top_button, uint8_t bottom_button, uint8_t step_pin, uint8_t dir_pin)
{
  uint8_t step_state = 0;
  while (!digitalRead(top_button))
  {
    digitalWrite(dir_pin, HIGH);
    step_state = !step_state;
    digitalWrite(step_pin, step_state);
    delayMicroseconds(LOOP_DELAY);
  }

  while (!digitalRead(bottom_button))
  {
    digitalWrite(dir_pin, LOW);
    step_state = !step_state;
    digitalWrite(step_pin, step_state);
    delayMicroseconds(LOOP_DELAY);
  }
}

void handleButtonDualMotor(uint8_t top_button, uint8_t bottom_button, uint8_t step_pin_a, uint8_t dir_pin_a, uint8_t step_pin_b, uint8_t dir_pin_b, bool sync)
{
  uint8_t step_state = 0;
  while (!digitalRead(top_button))
  {
    digitalWrite(dir_pin_a, HIGH);
    digitalWrite(dir_pin_b, sync ? HIGH : LOW);

    step_state = !step_state;
    digitalWrite(step_pin_a, step_state);
    digitalWrite(step_pin_b, step_state);
    delayMicroseconds(LOOP_DELAY);
  }

  while (!digitalRead(bottom_button))
  {
    digitalWrite(dir_pin_a, LOW);
    digitalWrite(dir_pin_b, sync ? LOW : HIGH);

    step_state = !step_state;
    digitalWrite(step_pin_a, step_state);
    digitalWrite(step_pin_b, step_state);
    delayMicroseconds(LOOP_DELAY);
  }
}

void loop()
{
  // BASE ROLL
  handleButtonSingleMotor(BASE_ROLL_TOP, BASE_ROLL_BOTTOM, BASE_ROLL_STEP, BASE_ROLL_DIR);
  // BASE PITCH
  handleButtonSingleMotor(BASE_PITCH_TOP, BASE_PITCH_BOTTOM, BASE_PITCH_STEP, BASE_PITCH_DIR);
  // ELBOW
  handleButtonSingleMotor(ELBOW_TOP, ELBOW_BOTTOM, ELBOW_STEP, ELBOW_DIR);
  // WRIST ROLL
  handleButtonSingleMotor(WRIST_ROLL_TOP, WRIST_ROLL_BOTTOM, WRIST_ROLL_STEP, WRIST_ROLL_DIR);
  // WRIST PITCH
  handleButtonSingleMotor(WRIST_PITCH_TOP, WRIST_PITCH_BOTTOM, WRIST_PITCH_STEP, WRIST_PITCH_DIR);
  // CLAW
  handleButtonSingleMotor(CLAW_TOP, CLAW_BOTTOM, CLAW_STEP, CLAW_DIR);
}

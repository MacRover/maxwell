// 345us pulse width
// 1/345, 2.893khz
// 1/350, 2.85khz
double del = (345.0 / 8.0) * (18.0 / 12.0);

int ON_SIGNAL[] = {1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,  0, 0, 1, 1, 1, 0, 1};
int OFF_SIGNAL[] = { 1,1,1,0,1,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,1,1,0,1,0,0,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,0,0,0,1,1,1,0,1,0,0,0,1,1,1,0,1,1,1,0,1,1,1,0,1,1,1,0,1,0,0,0,1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1};

int length = 96;
//int length = sizeof(ON_SIGNAL) / sizeof(ON_SIGNAL[0]);

int READING[sizeof(ON_SIGNAL) / sizeof(ON_SIGNAL[0])] = {0};

int counter = 4;
int DOPin = 3;
int RelayPin = 2;

void setup() {
  Serial.begin(115200);
  pinMode(DOPin, INPUT);
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);
}

int prev_raw = 0;
void loop() {


  if ( memcmp( (const void *)ON_SIGNAL, (const void *)READING, sizeof(ON_SIGNAL)) == 0)
  {
    Serial.println("ON");
    digitalWrite(RelayPin, HIGH);
  }

  if ( memcmp( (const void *)OFF_SIGNAL, (const void *)READING, sizeof(OFF_SIGNAL)) == 0)
  {
    Serial.println("OFF");
    digitalWrite(RelayPin, LOW);
  }

  int raw = digitalRead(DOPin);
  // Serial.println(raw);
  if (raw == prev_raw) {
    counter++;
    if (counter > 3) {
      //push this into the window
      pushtoReading(raw);
          //  Serial.println(raw);
      counter = 0;
    }
  } else if (raw != prev_raw) {
    counter = 0;
  }


  //Serial.print ("counter: ");
  // Serial.println( counter);
  prev_raw = raw;
  delayMicroseconds(del);

}

void pushtoReading(int newVal) {

  for (int i = 0; i < length; i++) {
    READING[i] = READING[i + 1];
  }
  READING[length] = newVal;
}
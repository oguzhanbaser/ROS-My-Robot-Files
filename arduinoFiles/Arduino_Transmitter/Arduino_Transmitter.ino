#define LED1 7
#define LED2 8
#define LED3 13

#define SWPIN1 A0
#define SWPIN2 A1
#define SWPIN3 A2

#define P_MOTOR_1 A6
#define P_MOTOR_2 A7

#define P_SERVO_X A4
#define P_SERVO_Y A5

#define MAX_HIZ_POT A3

#define BUTTON 4
#define SWITCH 2

unsigned long lastTime = 0;
bool servoStat = false;

void setup()
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);

  pinMode(SWPIN1, INPUT);
  digitalWrite(SWPIN1, HIGH);
  pinMode(SWPIN2, INPUT);
  digitalWrite(SWPIN2, HIGH);
  pinMode(SWPIN3, INPUT);
  digitalWrite(SWPIN3, HIGH);

  pinMode(BUTTON, INPUT);
  pinMode(LED3, OUTPUT);

  attachInterrupt(0, kesme, RISING);

  Serial.begin(9600);
}

void kesme()
{
  servoStat = !servoStat;
  digitalWrite(LED3, servoStat);
}

void loop()
{
  int val1 = analogRead(P_MOTOR_1);
  int val2 = analogRead(P_MOTOR_2);

  int valx = analogRead(P_SERVO_X);
  int valy = analogRead(P_SERVO_Y);

  int maxHiz = analogRead(MAX_HIZ_POT);
  maxHiz = map(maxHiz, 0, 1023, 0, 255);

  valx = map(valx, 0, 1024, 10, 170);
  valy = map(valy, 0, 1024, 160, 20);

  if(valy < 40) valy = 40;

  /*
  if (digitalRead(BUTTON))
  {
    servoStat = !servoStat;
    while (digitalRead(BUTTON)) delay(10);
  }*/

  if (!digitalRead(SWPIN1))
  {
    if (millis() - lastTime > 100)
    {
      Serial.print("#|");
      Serial.print(val1);
      Serial.print("|");
      Serial.print(val2);
      Serial.print("|");
      Serial.print(valx);
      Serial.print("|");
      Serial.print(valy);
      Serial.print("|");
      Serial.print(servoStat);
      Serial.print("|");
      Serial.print(maxHiz);
      Serial.println("|");

      lastTime = millis();
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
    }
  } else if (!digitalRead(SWPIN2))
  {
    if (millis() - lastTime > 1000)
    {
      Serial.println("&0|");
      lastTime = millis();
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
    }
  } else if (!digitalRead(SWPIN3))
  {
    if (millis() - lastTime > 1000)
    {
      lastTime = millis();
      digitalWrite(LED1, !digitalRead(LED1));
      digitalWrite(LED2, !digitalRead(LED2));
    }
  }
}


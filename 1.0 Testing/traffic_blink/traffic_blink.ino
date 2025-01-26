int animationSpeed = 400;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop()
{
  
  digitalWrite(13, HIGH);
  delay(animationSpeed); // Wait for animationSpeed millisecond(s)
  digitalWrite(13, LOW);
  delay(animationSpeed); // Wait for animationSpeed millisecond(s)
  digitalWrite(12, HIGH);
  delay(200); // Wait for animationSpeed millisecond(s)
  digitalWrite(12, LOW);
  delay(200); // Wait for animationSpeed millisecond(s)
  digitalWrite(12, HIGH);
  delay(200); // Wait for animationSpeed millisecond(s)
  digitalWrite(12, LOW);
  delay(animationSpeed); // Wait for animationSpeed millisecond(s)
  digitalWrite(11, HIGH);
  delay(animationSpeed); // Wait for animationSpeed millisecond(s)
  digitalWrite(11, LOW);
  delay(animationSpeed); // Wait for animationSpeed millisecond(s)
  digitalWrite(12, HIGH);
  delay(animationSpeed); // Wait for animationSpeed millisecond(s)
  digitalWrite(12, LOW);
  delay(animationSpeed); // Wait for animationSpeed millisecond(s)
}

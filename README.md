# Hnd-Ges-Control-
#include <Wire.h>
#include <LCD-I2C.h>

#define FLEX1_PIN A0  // Flex Sensor 1 Pin
#define FLEX2_PIN A1  // Flex Sensor 2 Pin
#define FLEX3_PIN A2  // Flex Sensor 3 Pin
#define BUZZER_PIN 9  // Buzzer Pin

LCD_I2C lcd(0x27, 16, 2);  // Initialize LCD (16x2)

// Constants
#define READINGS_COUNT 50 // Number of readings to average
#define READ_INTERVAL 10  // Time between readings in milliseconds

// Arrays to store sensor readings
int flex1_readings[READINGS_COUNT] = {0};
int flex2_readings[READINGS_COUNT] = {0};
int flex3_readings[READINGS_COUNT] = {0};

void setup() {
  Serial.begin(9600);          // Initialize Serial Monitor
  Wire.begin();                // Initialize I2C communication
  lcd.begin(&Wire);            // Initialize LCD
  lcd.display();               // Turn on LCD display
  lcd.backlight();             // Turn on LCD backlight
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output
  digitalWrite(BUZZER_PIN, LOW);

  lcd.setCursor(0, 0);
  lcd.print("Flex Test Init");
  delay(2000);
  lcd.clear();
}

void loop() {
  // Reset readings index
  int index = 0;

  // Collect readings rapidly over 5 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    flex1_readings[index] = analogRead(FLEX1_PIN);
    flex2_readings[index] = analogRead(FLEX2_PIN);
    flex3_readings[index] = analogRead(FLEX3_PIN);

    // Debug: Print current readings
    Serial.print("F1: "); Serial.print(flex1_readings[index]);
    Serial.print(" | F2: "); Serial.print(flex2_readings[index]);
    Serial.print(" | F3: "); Serial.println(flex3_readings[index]);

    index++;
    if (index >= READINGS_COUNT) index = 0; // Loop back if we exceed the array size

    delay(READ_INTERVAL);
  }

  // Calculate average for each sensor
  float avgFlex1 = calculateAverage(flex1_readings);
  float avgFlex2 = calculateAverage(flex2_readings);
  float avgFlex3 = calculateAverage(flex3_readings);

  // Determine which flex sensor is bent the most
  lcd.clear();
  lcd.setCursor(0, 0);

  if (avgFlex1 > avgFlex2 && avgFlex1 > avgFlex3) {
    lcd.print("GO AHEAD");
    Serial.println("GO AHEAD");
  } else if (avgFlex2 > avgFlex1 && avgFlex2 > avgFlex3) {
    lcd.print("STOP");
    Serial.println("STOP");
  } else if (avgFlex3 > avgFlex1 && avgFlex3 > avgFlex2) {
    lcd.print("CALL 112");
    Serial.println("CALL 112");

    // Activate buzzer for "CALL 112"
    for (int i = 0; i < 3; i++) { // Buzzer beeps 3 times
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      delay(250);
    }
  } else {
    lcd.print("UNKNOWN STATE");
    Serial.println("UNKNOWN STATE");
  }

  // Debugging: Print average values
  Serial.println("Average Flex 1: " + String(avgFlex1));
  Serial.println("Average Flex 2: " + String(avgFlex2));
  Serial.println("Average Flex 3: " + String(avgFlex3));
  Serial.println();
}

// Function to calculate average of an array
float calculateAverage(int readings[]) {
  long sum = 0;
  for (int i = 0; i < READINGS_COUNT; i++) {
    sum += readings[i];
  }
  return (float)sum / READINGS_COUNT;
}

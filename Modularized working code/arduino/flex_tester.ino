/*
 * Flex Sensor Final Calibration (Parallel)
 * Pin: 33 (ADC1)
 * CIRCUIT: 3.3V -> Flex -> Pin 33 -> [Parallel Resistors 2.35k] -> GND
 */

const int FLEX_PIN = 33;
int minRaw = 4095;
int maxRaw = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Ensure we use full 0-3.3V range sensitivity
  analogSetAttenuation(ADC_11db); 
  
  Serial.println("--- Final Flex Calibration ---");
  Serial.println("1. Finger STRAIGHT: Note 'Raw' value.");
  Serial.println("2. Finger BENT: Note 'Raw' value.");
  Serial.println("------------------------------");
}

void loop() {
  int rawValue = analogRead(FLEX_PIN);
  
  // Auto-track the range we see
  if (rawValue > 0 && rawValue < minRaw) minRaw = rawValue;
  if (rawValue > maxRaw) maxRaw = rawValue;

  Serial.print("Current: ");
  Serial.print(rawValue);
  Serial.print("\t[Min: ");
  Serial.print(minRaw);
  Serial.print(" | Max: ");
  Serial.print(maxRaw);
  Serial.println("]");

  delay(100); 
}

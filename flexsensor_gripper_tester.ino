const int FLEX_PIN = A0;
// YOUR calibrated values
int FLAT_ADC = 450;
int BENT_ADC = 1000;

void setup() {
  Serial.begin(115200);
  pinMode(FLEX_PIN, INPUT);
  delay(1000);
  
  Serial.println("READY");
}

void loop() {
  int raw = analogRead(FLEX_PIN);
  raw = constrain(raw, FLAT_ADC, BENT_ADC);
  
  // Normalize to 0.0 - 1.0
  float normalized = (float)(raw - FLAT_ADC) / (float)(BENT_ADC - FLAT_ADC);
  
  // Apply exponential curve (power of 2.5)
  float curved = pow(normalized, 2.5);
  
  // Convert to percentage (0-100)
  int percentage = (int)(curved * 100.0);
  percentage = constrain(percentage, 0, 100);
  
  // Send to Python
  Serial.print("ANGLE:");
  Serial.println(percentage);
  
  delay(50);
}
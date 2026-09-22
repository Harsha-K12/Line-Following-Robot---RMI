#define S1_PIN 34
#define S2_PIN 35
#define S3_PIN 32
#define S4_PIN 33
#define S5_PIN 25
#define S6_PIN 26
#define S7_PIN 27
#define S8_PIN 14

int S1,S2,S3,S4,S5,S6,S7,S8;

void setup() {
Serial.begin(9600);
pinMode(S1_PIN,INPUT);
pinMode(S2_PIN,INPUT);
pinMode(S3_PIN,INPUT);
pinMode(S4_PIN,INPUT);
pinMode(S5_PIN,INPUT);
pinMode(S6_PIN,INPUT);
pinMode(S7_PIN,INPUT);
pinMode(S8_PIN,INPUT);
}

void loop() {
S1=analogRead(S1_PIN);
S2=analogRead(S2_PIN);
S3=analogRead(S3_PIN);
S4=analogRead(S4_PIN);
S5=analogRead(S5_PIN);
S6=analogRead(S6_PIN);
S7=analogRead(S7_PIN);
S8=analogRead(S8_PIN);
Serial.print(" S1: ");
Serial.print(S1);
Serial.print(" S2: ");
Serial.print(S2);
Serial.print(" S3: ");
Serial.print(S3);
Serial.print(" S4: ");
Serial.print(S4);
Serial.print(" S5: ");
Serial.print(S5);
Serial.print(" S6: ");
Serial.print(S6);
Serial.print(" S7: ");
Serial.print(S7);
Serial.print(" S8: ");
Serial.println(S8);
delay(250);
}

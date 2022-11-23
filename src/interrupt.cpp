int LED= 21;
int contador=0;
hw_timer_t *My_timer = NULL;
void IRAM_ATTR onTimer(){
  contador++;
digitalWrite(LED, !digitalRead(LED));
}
void setup() {
  Serial.begin(9600);
pinMode(LED, OUTPUT);
My_timer = timerBegin(0, 80, true);
timerAttachInterrupt(My_timer, &onTimer, true);
timerAlarmWrite(My_timer, 1000000, true);
timerAlarmEnable(My_timer); //Just Enable
}
void loop() {
  Serial.println(contador);
}
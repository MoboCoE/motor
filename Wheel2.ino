#define IN1 PA0
#define IN2 PA1
#define ENA PA5

#define LED PC13
#define TIM3_CH1 PA6
#define TIM3_CH2 PA7
#define TIM4_CH1 PB6
#define TIM4_CH2 PB7

float setpoint = 2;
float error = 0;
float Kp = 70;
float Ki = 0.8;
float Kd = 0;

float sum = 0;
float prevError = 0;

float speed = 0;
float prevSpeed = 0;
float PWMvalue = 0;

float GearRatio = 20;
float Pulse = 44;
float Wheel = 65;
float LoopFreq = 100;
float Vel = -200; 

long V3H, V3L, CNT3;

void timer3Int(void);
void timer4Int(void);

TIM_TypeDef *inst4 = TIM4;
HardwareTimer *timer4;
TIM_TypeDef *inst3 = TIM3;
HardwareTimer *timer3;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(TIM3_CH1, INPUT);
  pinMode(TIM3_CH2, INPUT);
  pinMode(TIM4_CH1, INPUT);
  pinMode(TIM4_CH2, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);


  timer3 = new HardwareTimer(inst3);
  timer4 = new HardwareTimer(inst4);

  // config GPIO to alternate function for timer4 (CH1=PB6, CH2=PB7)
  //GPIOB->AFRL = 0b0010 0010 0000 0000 0000 0000 0000 0000
  GPIOA->AFR[0] = (GPIOA->AFR[0] & 0x00FFFFFF) + 0x22000000;
  GPIOB->AFR[0] = (GPIOB->AFR[0] & 0x00FFFFFF) + 0x22000000;
  // config GPIO to AF (alternate function mode)
  // MODER7 and MODER6 = 0xb10
  GPIOA->MODER = (GPIOA->MODER & 0xFFFF0FFF) + 0x0000A000;
  GPIOB->MODER = (GPIOB->MODER & 0xFFFF0FFF) + 0x0000A000;

  //CC1S=’01’ (TIMx_CCMR1 register, TI1FP1 mapped on TI1).
  // CC1S = last 2 bits
  //CC2S=’01’ (TIMx_CCMR2 register, TI1FP2 mapped on TI2).
  // CC2S = bit 9 and 8
  TIM3->CCMR1 = (TIM3->CCMR1 & 0xFFFFFCFC) + 0b0100000001;
  TIM4->CCMR1 = (TIM4->CCMR1 & 0xFFFFFCFC) + 0b0100000001;

  //SMS=’011’ (TIMx_SMCR register, both inputs are active on both rising and fallingedges).
  // SMS = last 3 bits of SMCR
  TIM3->SMCR = (TIM3->SMCR & 0xFFFFFFF8) + 0b011;
  TIM4->SMCR = (TIM4->SMCR & 0xFFFFFFF8) + 0b011;
  //CC1P=’0’, CC1NP=’0’, and IC1F = ‘0000’ (TIMx_CCER register, TI1FP1 non-inverted,TI1FP1=TI1).
  //CC2P=’0’, CC2NP=’0’, and IC2F = ‘0000’ (TIMx_CCER register, TI1FP2 non-inverted,TI1FP2= TI2).
  // no need to do anything here becase they are all zero's after reset

  timer3->attachInterrupt(timer3Int);
  timer4->attachInterrupt(timer4Int);

  //CEN=’1’ (TIMx_CR1 register, Counter enabled).
  // CEN is last bit of CR1
  TIM3->CR1 = (TIM3->CR1 & 0xFFFE) + 1;
  TIM4->CR1 = (TIM4->CR1 & 0xFFFE) + 1;
}

void loop() {
  digitalWrite(LED, HIGH);
  float pulse_per_mm = (GearRatio * Pulse) /(Wheel*3.14);
  float mm_per_pulse = (Wheel*3.14) / (GearRatio * Pulse);
  float pulse_per_sec = Vel * pulse_per_mm;
  float pulse_per_loop = pulse_per_sec / LoopFreq;

  int v3, v4;

  v3 = timer3->getCount();
  v4 = timer4->getCount();

  speed = v3 - prevSpeed;
  prevSpeed = v3;
  error = pulse_per_loop - speed;

  sum += error;
  if (sum > 500) {
    sum = 500;
  }
  if (sum < -500) {
    sum = -500;
  }
  PWMvalue = (Kp * error) + (Ki * sum);

  if (PWMvalue >= 0){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
  }
  else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      PWMvalue = PWMvalue * -1;
  }

  PWMvalue = map(PWMvalue, 0, 1000, 0, 255);
  analogWrite(ENA, (int)PWMvalue);
  CNT3 = timer3 -> getCount() + (V3H << 16);
  Serial.print("TIM3: ");
  Serial.print(v3);
  Serial.print("  CNT3: ");
  Serial.print(CNT3);
  Serial.print("       PWM: ");
  Serial.print((int)PWMvalue);
  Serial.print(" Speed: ");
  Serial.print(speed);
  Serial.print("  Error: ");
  Serial.println(error);
  //Serial.print("  TIM4: ");
  //Serial.println(v4);

  // analogWrite(LED, map(v4, 0, 65536, 0, 255));
  //analogWrite(ENA, 100);

  digitalWrite(LED, LOW);
  delay(10);
}

void timer3Int(void) {
  //digitalWrite(LED, !digitalRead(LED));
  if (TIM3->CR1 & 0x10){
    V3H--;
  }
  else{
    V3H++;
  }
}

void timer4Int(void) {
  //digitalWrite(LED, !digitalRead(LED));
}

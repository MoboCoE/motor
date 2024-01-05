
#define LED      PC13
#define TIM3_CH1 PA6
#define TIM3_CH2 PA7
#define TIM4_CH1 PB6
#define TIM4_CH2 PB7
//      L298     STM32
#define IN3      PA0
#define IN4      PA1
#define PWM_A5   PA5   

void timer3Int(void);
void timer4Int(void);

TIM_TypeDef *inst4 = TIM4;

HardwareTimer *timer4;
TIM_TypeDef *inst3 = TIM3;
HardwareTimer *timer3;

float setpoint = 2;
float error = 0;
float Kp = 70;
float Ki = 0.8;
float Kd = 100;
float old_error = 0;
float sum_error = 0;
float speed = 0;
float last_cnt = 0;
float pwm_value = 0;
float diff_error = 0;
float gear_ratio = 20;
float pulse_per_rev = 44;
float wheel_diameter = 65;
int loop_freq = 100; // 100Hz -> T = 10ms
const float Pi = 3.14159;
int velocity = -200; // ความเร็วที่ต้องการ mm/s
long V3H, V3L, V4H, V4L; //long -> 32bit
long CNT3, CNT4;


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(TIM3_CH1, INPUT);
  pinMode(TIM3_CH2, INPUT);
  pinMode(TIM4_CH1, INPUT);
  pinMode(TIM4_CH2, INPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWM_A5, OUTPUT);


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
  TIM3->CCMR1  = (TIM3->CCMR1 & 0xFFFFFCFC) + 0b0100000001;
  TIM4->CCMR1  = (TIM4->CCMR1 & 0xFFFFFCFC) + 0b0100000001;
  
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

  int CNT3, CNT4;

  // CNT3 = timer3->getCount();
  // CNT4 = timer4->getCount();
  CNT3 = timer3->getCount() + (V3H << 16); //V3 บนshift leftไป 16 bit
  V4L = timer4->getCount();
  CNT4 = V4L + (V4H << 16);

  float mm_per_pulse = (wheel_diameter*Pi)/(gear_ratio*pulse_per_rev);
  float pulse_per_mm = 1 / mm_per_pulse;
  float pulse_per_sec = velocity*pulse_per_mm;
  float pulse_per_loop = pulse_per_sec/loop_freq;

  setpoint = pulse_per_loop;

  speed = CNT4 - last_cnt;
  last_cnt = CNT4;
  error = setpoint - speed;
  old_error = error;
  sum_error += error;
  diff_error = error-old_error;

  if (sum_error > 500){
    sum_error = 500;
  }
  if(sum_error<-500){
    sum_error = -500;
  }

  pwm_value = Kp*error + Ki*sum_error + Kd*diff_error;
  // Serial.print("1: ");
  // Serial.println(pwm_value);
  pwm_value = map(pwm_value, 0, 1000, 0, 255); //ไม่ให้มากกว่า 1000
  // Serial.print("2: ");
  // Serial.println(pwm_value);

  if (pwm_value >= 0){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(PWM_A5, (int)pwm_value);
  }
  else{
    pwm_value = -pwm_value;
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(PWM_A5, (int)pwm_value);
  }
  // Serial.print("Error  ");
  // Serial.print(error);
  // Serial.print("  Speed:  ");
  // Serial.print(speed);
  // Serial.print("  PWM:  ");
  // Serial.print((int)pwm_value);
  // Serial.print("  CNT4:   ");
  // Serial.print(CNT4);  
  // Serial.print("  V4L:   ");
  // Serial.print(V4L);  
  // Serial.print("  V4H:   ");
  // Serial.println(V4H);  

  // Serial.print("TIM3: ");
  // Serial.print(CNT3);
 // analogWrite(LED, map(CNT4, 0, 65536, 0, 255));
  digitalWrite(LED, LOW);
  delay(10);
}

void timer3Int(void) {
  if (TIM3_CH1 & 0x10){
    // Down Count
    V3H--;
  }
  else{
    // Up Count
    V3H++;
  }
  // digitalWrite(LED, !digitalRead(LED));
}

void timer4Int(void) {
  // digitalWrite(LED, !digitalRead(LED));
  if (TIM4->CR1 & 0x10){
    // Down Count
    V4H--;
  }
  else{
    // Up Count
    V4H++;
  }
}

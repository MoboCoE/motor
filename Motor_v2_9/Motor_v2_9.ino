#include "STM32TimerInterrupt.h"
#include <ros.h>
#include <STM32_ISR_Timer.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Int16.h>
#include <ros/time.h>

#if !(defined(STM32F0) || defined(STM32F1) || defined(STM32F2) || defined(STM32F3) || defined(STM32F4) || defined(STM32F7) || defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32H7) || defined(STM32G0) || defined(STM32G4) || defined(STM32WB) || defined(STM32MP1) || defined(STM32L5))

#error This code is designed to run on STM32F/L/H/G/WB/MP1 platform! Please check your Tools->Board setting.

#endif

#define TIMER_INTERVAL_MS 100
#define HW_TIMER_INTERVAL_MS 50

STM32Timer ITimer(TIM1);
STM32_ISR_Timer ISR_Timer;

#define TIMER_INTERVAL_ms 10L

void TimerHandler() {
  ISR_Timer.run();
}

#define IN1 PA2
#define IN2 PA1
#define ENA PA5

#define IN3 PB2
#define IN4 PB1
#define ENB PB0

#define LED PC13
#define TIM3_CH1 PA6
#define TIM3_CH2 PA7
#define TIM4_CH1 PB6
#define TIM4_CH2 PB7

// ros
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist>
cmd_vel_sub("cmd_vel", &cmd_vel_callback);
geometry_msgs::Vector3Stamped speed_msg;  //create a "speed_msg" ROS message​

std_msgs::Int16 lwheel_mgs;  // these 2 int16 variables are used to sent encoder counts to differential drive package​
std_msgs::Int16 rwheel_mgs;
ros::Publisher lwheel_pub("lwheel", &lwheel_mgs);
ros::Publisher rwheel_pub("rwheel", &rwheel_mgs);

HardwareSerial Serial1(PA10, PA9);
#define USE_STM32_HW_SERIAL

// Right Motor
float setpoint_R = 2;
float error_R = 0;
float Kp_R = 70;
float Ki_R = 0.8;
float Kd_R = 0;

float sum_R = 0;
float prevError_R = 0;

float speed_R = 0;
float prevSpeed_R = 0;
float PWMvalue_R = 0;

float GearRatio_R = 20;
float Pulse_R = 44;
float Wheel_R = 65;
float LoopFreq_R = 100;
volatile float Vel_R = -200;

volatile long v3, v4;
long V3H, V3L, CNT3;
long V4H, V4L, CNT4;

// Left Motor
float setpoint_L = 2;
float error_L = 0;
float Kp_L = 70;
float Ki_L = 0.8;
float Kd_L = 0;

float sum_L = 0;
float prevError_L = 0;

float speed_L = 0;
float prevSpeed_L = 0;
float PWMvalue_L = 0;

float GearRatio_L = 20;
float Pulse_L = 44;
float Wheel_L = 65;
float LoopFreq_L = 100;
volatile float Vel_L = -200;

void timer3Int(void);
void timer4Int(void);

void driveMotor(void);

TIM_TypeDef *inst4 = TIM4;
HardwareTimer *timer4;
TIM_TypeDef *inst3 = TIM3;
HardwareTimer *timer3;

void setup() {
  Serial.begin(115200);
  //ros init
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(lwheel_pub);
  nh.advertise(rwheel_pub);


  pinMode(LED, OUTPUT);
  pinMode(TIM3_CH1, INPUT);
  pinMode(TIM3_CH2, INPUT);
  pinMode(TIM4_CH1, INPUT);
  pinMode(TIM4_CH2, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);


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

  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler)) {
    Serial.print(F("Starting ITimer OK, millis() = "));
    Serial.println(millis());
  } else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  // Just to demonstrate, don't use too many ISR Timers if not absolutely necessary
  // You can use up to 16 timer for each ISR_Timer
  ISR_Timer.setInterval(TIMER_INTERVAL_ms, driveMotor);
}

void loop() {
  nh.spinOnce(); 
  Serial.print("R: ");
  Serial.print(Vel_R);
  Serial.print(" L: ");
  Serial.println(Vel_L);
  //Serial.print("TIM3: ");
  //Serial.print(v3);
  //Serial.print("  CNT3: ");
  //Serial.print(CNT3);
  //Serial.print("       PWM: ");
  //Serial.print((int)PWMvalue_R);
  //Serial.print(" Speed: ");
  //Serial.print(speed_R);
  //Serial.print("  Error: ");
  //Serial.println(error_R);
  //Serial.print("  TIM4: ");
  //Serial.println(v4);
}

void driveMotor() {
  digitalWrite(LED, HIGH);

  float pulse_per_mm_R = (GearRatio_R * Pulse_R) / (Wheel_R * 3.14);
  float mm_per_pulse_R = (Wheel_R * 3.14) / (GearRatio_R * Pulse_R);
  float pulse_per_sec_R = Vel_R * pulse_per_mm_R;
  float pulse_per_loop_R = pulse_per_sec_R / LoopFreq_R;

  float pulse_per_mm_L = (GearRatio_L * Pulse_L) / (Wheel_L * 3.14);
  float mm_per_pulse_L = (Wheel_L * 3.14) / (GearRatio_L * Pulse_L);
  float pulse_per_sec_L = Vel_L * pulse_per_mm_L;
  float pulse_per_loop_L = pulse_per_sec_L / LoopFreq_L;

  v3 = timer3->getCount();
  v3 = v3 + (V3H << 16);
  v4 = timer4->getCount();
  v4 = v4 + (V4H << 16);

  speed_R = v3 - prevSpeed_R;
  prevSpeed_R = v3;
  error_R = pulse_per_loop_R - speed_R;
  sum_R += error_R;

  speed_L = v4 - prevSpeed_L;
  prevSpeed_L = v4;
  error_L = pulse_per_loop_L - speed_L;
  sum_L += error_L;

  rwheel_mgs.data = speed_R;
  lwheel_mgs.data = speed_L;
  lwheel_pub.publish(&lwheel_mgs);
  rwheel_pub.publish(&rwheel_mgs);

  if (sum_R > 500) {
    sum_R = 500;
  }
  if (sum_R < -500) {
    sum_R = -500;
  }

  if (sum_L > 500) {
    sum_L = 500;
  }
  if (sum_L < -500) {
    sum_L = -500;
  }

  PWMvalue_R = (Kp_R * error_R) + (Ki_R * sum_R);
  PWMvalue_L = (Kp_L * error_L) + (Ki_L * sum_L);

  if (PWMvalue_R >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    PWMvalue_R = PWMvalue_R * -1;
  }

  if (PWMvalue_L >= 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    PWMvalue_L = PWMvalue_L * -1;
  }

  PWMvalue_R = map(PWMvalue_R, 0, 1000, 0, 255);
  PWMvalue_L = map(PWMvalue_L, 0, 1000, 0, 255);

  analogWrite(ENA, (int)PWMvalue_R);
  analogWrite(ENB, (int)PWMvalue_L);

  //CNT3 = timer3->getCount() + (V3H << 16);


  // analogWrite(LED, map(v4, 0, 65536, 0, 255));
  //analogWrite(ENA, 100);

  digitalWrite(LED, LOW);
}

void timer3Int(void) {
  //digitalWrite(LED, !digitalRead(LED));
  if (TIM3->CR1 & 0x10) {
    V3H--;
  } else {
    V3H++;
  }
}

void timer4Int(void) {
  //digitalWrite(LED, !digitalRead(LED));
  if (TIM4->CR1 & 0x10) {
    V4H--;
  } else {
    V4H++;
  }
}



void cmd_vel_callback(const geometry_msgs::Twist& msg)
{

  double speed_ang = msg.angular.z;

  double speed_lin = msg.linear.x;

  Vel_R = 200 * (speed_lin + (speed_ang * 65/2.0));

  Vel_L = 200  * (speed_lin - (speed_ang * 65/2.0));

}

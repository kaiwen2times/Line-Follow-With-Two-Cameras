#ifndef PWM_H_
#define PWM_H_

void motor1Write(unsigned int DutyCycle, int dir);
void motor2Write(unsigned int DutyCycle, int dir);
void servoWrite(float DutyCycle);
void InitPWM(void);

#endif /* PWM_H_ */

// Credit to : Dung, Ngo Van ***** 13TDH2//Danang Universtity of Technology (DUT)
// Final thesis PIC 18F4550 source code
// * Calculate PWM value for DC motor driver to make motor speed meet refer speed
// * Receive PID parameter and refer speed from PC or other device via RS232 communication
// * Using RTOS 
// * Feedback speed to enhance control quality

#include <18F4550.h>
#include <ieeefloat.c>
#include <FuzzyLib.c>

// configuration check fuses list for details
#fuses HSPLL,NOWDT,NOPROTECT,NOLVP,NODEBUG,USBDIV,PLL5,CPUDIV1,VREGEN

#use delay(clock=48MHz)

#use rs232(UART1,baud=9600,PARITY=N,BITS=8,RECEIVE_BUFFER=30)

#define Ts 0.05

#use rtos(timer=3,minor_cycle=25ms)


unsigned int16 pulse = 0,pulseSum = 0,pulseSum2 = 0;
int32 ref_speed;                                                      //RS232 Data receive variable
int8 *p_ref_speed=&ref_speed,count = 0,count2 = 0;                                   // pointers RS232 to  Data receive variable

double FKp,FKi,FKd = 0,Fref_speed,A,B,C,out_speed,er_speed = 0,er_speed2,d_er_speed;
double error_pulse[3],ref_pulse,PID_val_res;
double pwm_val_o=0,pwm_val_n;

TriMem ErFzySet[3],DErFzySet[3];
TriMem KpFzySet[3],KiFzySet[3],KdFzySet[3];
Out_range RangeKp,RangeKi,RangeKd;

#task(rate=500ms,max=5ms)
void To_RS232()                                                      //_Send the current speed to the PC
{  
out_speed = (double)pulseSum2*3.3/(double)count2;
pulseSum2=0;
count2 =0;

   printf("%.1f|%.1f|%.1f|%.1f|%.4f|%u",Fref_speed,out_speed,FKp,FKi,Fkd,(int)pwm_val_n);
}

#task(rate=500ms,max=10ms)
void pwm_value_read()                                                //_Receive ref_speed
{  
   if(Kbhit()) 
   {
   p_ref_speed[0] = getc();
   p_ref_speed[1] = getc();
   p_ref_speed[2] = getc();
   p_ref_speed[3] = getc();
   
   Fref_speed = f_IEEEtoPIC(ref_speed);
   
   ref_pulse = Fref_speed*0.3025;
   
//!   A = (FKp + FKi*Ts/2 + FKd/Ts);
//!   B = (-FKp + FKi*Ts/2 - 2*FKd/Ts);
//!   C = FKd/Ts;

   printf(" !!! ");
   }
}

#task(rate = 200ms,max = 25ms)
void Fzy_PID()
{
out_speed = (double)pulseSum*3.3/(double)count;
pulseSum=0;
count =0;
er_speed2 = er_speed;
er_speed = out_speed - Fref_speed;
d_er_speed = (er_speed - er_speed2)*2;
er_speed = (er_speed>0) ? er_speed : -er_speed; 
d_er_speed = (d_er_speed>0) ? d_er_speed : -d_er_speed; 
for(int i=0;i<6;i++)
   {
      DErFzySet[i].degree = Mem_degree(DErFzySet[i],d_er_speed);
      ErFzySet[i].degree = Mem_degree(ErFzySet[i],er_speed);
   }
for(int k=0;k<3;k++)
   {
      KpFzySet[k].degree = 0;
      KiFzySet[k].degree = 0;
      KdFzySet[k].degree = 0;
   }
//Rule set
Rule2_1_Min(&ErFzySet[0],&DErFzySet[0],&KpFzySet[1]);
Rule2_1_Min(&ErFzySet[0],&DErFzySet[1],&KpFzySet[1]);
Rule2_1_Min(&ErFzySet[0],&DErFzySet[2],&KpFzySet[0]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[0],&KpFzySet[2]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[1],&KpFzySet[2]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[2],&KpFzySet[1]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[0],&KpFzySet[2]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[1],&KpFzySet[2]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[2],&KpFzySet[2]);

Rule2_1_Min(&ErFzySet[0],&DErFzySet[0],&KiFzySet[1]);
Rule2_1_Min(&ErFzySet[0],&DErFzySet[1],&KiFzySet[0]);
Rule2_1_Min(&ErFzySet[0],&DErFzySet[2],&KiFzySet[0]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[0],&KiFzySet[2]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[1],&KiFzySet[1]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[2],&KiFzySet[0]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[0],&KiFzySet[2]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[1],&KiFzySet[2]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[2],&KiFzySet[2]);

Rule2_1_Min(&ErFzySet[0],&DErFzySet[0],&KdFzySet[1]);
Rule2_1_Min(&ErFzySet[0],&DErFzySet[1],&KdFzySet[1]);
Rule2_1_Min(&ErFzySet[0],&DErFzySet[2],&KdFzySet[2]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[0],&KdFzySet[0]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[1],&KdFzySet[1]);
Rule2_1_Min(&ErFzySet[1],&DErFzySet[2],&KdFzySet[2]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[0],&KdFzySet[0]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[1],&KdFzySet[0]);
Rule2_1_Min(&ErFzySet[2],&DErFzySet[2],&KdFzySet[1]);
//end rule set

RangeKp = Get_Range(&KpFzySet[0],3);
FKp = Defuzy(&KpFzySet[0],3,RangeKp);
RangeKi = Get_Range(&KiFzySet[0],3);
FKi = Defuzy(&KiFzySet[0],3,RangeKi);
RangeKd = Get_Range(&KdFzySet[0],3);
FKd = Defuzy(&KdFzySet[0],3,RangeKd);
}

//!#task(rate = 300ms,max = 20ms)
//!void Defzy()
//!{
//!RangeKp = Get_Range(&KpFzySet[0],3);
//!FKp = Defuzy(&KpFzySet[0],3,RangeKp);
//!RangeKi = Get_Range(&KiFzySet[0],3);
//!FKi = Defuzy(&KiFzySet[0],3,RangeKi);
//!RangeKd = Get_Range(&KdFzySet[0],3);
//!FKd = Defuzy(&KdFzySet[0],3,RangeKd);
//!}
//!
//!#task(rate = 200ms,max = 20ms)
//!void DefzyKi()
//!{
//!RangeKi = Get_Range(&KiFzySet[0],4);
//!FKi = Defuzy(&KiFzySet[0],4,RangeKi);
//!}
//!
//!#task(rate = 200ms,max = 20ms)
//!void DefzyKd()
//!{
//!RangeKd = Get_Range(&KdFzySet[0],3);
//!FKd = Defuzy(&KdFzySet[0],4,RangeKd);
//!}

#task(rate=50ms,max=2ms)
void Counter0_get()                          //_Read counter , Sum up counter to later speed calculation
{  
   pulse = get_timer0();
   set_timer0(0);
   count++;
   count2++;
   pulseSum += pulse;
   pulseSum2 += pulse;
}

#task(rate=50ms ,max=2ms )                   
void Error_regulator()                       //_ Error regulator for later PID calculation
{
   error_pulse[0] = error_pulse[1];
   error_pulse[1] = error_pulse[2];
   error_pulse[2] = ref_pulse - (double)pulse;                           //_The current value will then be used to compare with the require pulse.
}

#task(rate=50ms,max=4ms)
void PID_calculator()                       //_ PID eqution
{  
   A = (FKp + FKi*Ts/2 + FKd/Ts);
   B = (-FKp + FKi*Ts/2 - 2*FKd/Ts);
   C = FKd/Ts;
   PID_val_res = A*error_pulse[2] + B*error_pulse[1] + C*error_pulse[0];
}

#task(rate=50ms,max=2ms)
void PWM_adjust()                            //_Change the duty cycle to meet the reference speed
{
   pwm_val_n = pwm_val_o + PID_val_res;
   if(pwm_val_n < 0) pwm_val_n =0;
   if(pwm_val_n >250) pwm_val_n = 250;
   set_pwm1_duty((int)pwm_val_n);
   pwm_val_o = pwm_val_n;
}


void main()
{
printf ("System initializing \r\n");
printf ("Operation Mode \r\n");
printf ("MCU auto Fuzzy-PID\r\n");
delay_ms(1500);

ErFzySet[0] = Create_Member(-80, 0,80 );
ErFzySet[1] = Create_Member(60, 160, 260);
ErFzySet[2] = Create_Member(200,400,400);

DErFzySet[0] = Create_Member(-80,0,80);
DErFzySet[1] = Create_Member(50,150,250);
DErFzySet[2] = Create_Member(200,400,400);

KpFzySet[0] = Create_Member(0.4,1,1.6); //small
KpFzySet[1] = Create_Member(1.2,1.8,2.4);   //medium
KpFzySet[2] = Create_Member(2.2,3.6,5); //large


KiFzySet[0] = Create_Member(2.4, 6, 9.6); //small
KiFzySet[1] = Create_Member(9, 11.7, 14.4);    //medium
KiFzySet[2] = Create_Member(13.2, 21.6, 30); //large


KdFzySet[0] = Create_Member(0, 0.005, 0.01);
KdFzySet[1] = Create_Member(0.005, 0.015, 0.025);
KdFzySet[2] = Create_Member(0.015, 0.03, 0.045);


// Tris set up
set_tris_a(0x10);
set_tris_c(0x81);
set_tris_b(0x00);
set_tris_d(0x00);
//
enable_interrupts(GLOBAL);  

setup_ccp1(CCP_PWM);                   //_Enable PWM mode for CCP1 using timer2 

setup_timer_2(T2_DIV_BY_16, 250, 16);   //_Setup need to be revised -(3kHz frequency of period)

setup_timer_0(RTCC_EXT_L_TO_H|RTCC_DIV_1);

output_high(PIN_D0);
output_low(PIN_D1);

delay_ms(200);

error_pulse[0] =0.00;
error_pulse[1] =0.00;
error_pulse[2] =0.00;
count = 0;
pulseSum =0;

set_timer0(0);

rtos_run();
}

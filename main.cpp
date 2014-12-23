 
#include "mbed.h"
#include "math.h"
#include "rtos.h"
#include "IMU10DOF.h"


void pressed();


DigitalOut led1(LED1);
DigitalOut led2(PA_6);
InterruptIn mybutton(USER_BUTTON);

//Ticker cache;
Timer run_time;
uint32_t rt_st=0;
uint32_t rt_sp=0;
IMU10DOF imu(PB_9, PB_8);
byte s[4];
int Debug_pt = 0;
// autopilot
float Qset[4];  // set point for desired attitude
float Qerr[4];  // error values - diff fron  desired - actual
float Qcur[4];  // current orientation
int State=0; //  {INIT=0 ,STANBY=1, AUTO=2, DODGE=3};
int lastState=0;


int var2=0;
AnalogIn pin3(PA_0);
  
void pressed()  // User button oressed interupt
{
    State++;
    State = (State % 3);
    //Debug_pt++;
    Debug_pt = (Debug_pt % 6);
    imu.pc.printf("\nButton pressed rt_st=%d  debug_pt=%d  New state=%d \n", rt_st,Debug_pt, State);
}


 void fill_cache()
 {
	 uint16_t var2 = pin3.read_u16();;
	 imu.pc.printf("Var2-%d\n\r",var2);
	
 }


void get_imu(void const *args)
{
	if(Debug_pt==4) imu.pc.printf("start get_imu ");
	float q[4];
	float ypr[3];
	float values[9];
  while (1) {
	
    imu.getValues(values);
    if(Debug_pt==5) imu.pc.printf("ACC: %04.0f %04.0f %04.0f GYR: %04.0f %04.0f %04.0f MAG: %04.0f %04.0f %04.0f \n\r",values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8] );
    imu.getYawPitchRoll(ypr);
    if(Debug_pt==6) imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f \n",ypr[0],ypr[1],ypr[2]);
    imu.getQ(q);
      for (int i=0;i<4;i++) Qcur[i]=q[i];
      if(Debug_pt==3) imu.pc.printf("Q=%+5.3f %+5.3f %+5.3f %+5.3f\n", q[0], q[1], q[2], q[3]);
    rt_st = run_time.read_us();
    Thread::wait(30);
    run_time.reset();
  }
    //imu.pc.printf("get_imu wait=%x  \n",Thread::wait(20)); 
    if(Debug_pt==1) imu.pc.printf("end get_imu ");
}
 
void led2_thread(void const *args) {
    while (true) {
        printf("Led2 on\n\r");
        led2 = !led2;
      //  imu.pc.printf("led2_th wait=%x  \n",Thread::wait(1000));
    }
}


void print_q(char * Name, float *q) // print a quaternion
{
	imu.pc.printf(" %s= %+5.3f %+5.3f %+5.3f %+5.3f ", Name, q[0], q[1], q[2], q[3]);
}

int main() {
	
	
	//sensor.set_priority(osPriorityAboveNormal);
	imu.pc.printf("Ceci est un Test\r\n");
	imu.init(true);
	imu.gyro.status(s);imu.pc.printf(" GYR: %04i %04i %04i %04i \n\r",s[0],s[1],s[2],s[3]);
    imu.gyro.init();
    wait(1);
    imu.gyro.status(s);imu.pc.printf(" GYR2: %x %x %x %x \n\r",s[0],s[1],s[2],s[3]);
wait(2);

    run_time.start(); 
    
	//cache.attach(&get_imu, 0.1);	 
	Thread sensor(get_imu);
  //  Thread thread(led2_thread);
  
	mybutton.fall(&pressed);  // This works on the correct pin
    
    while (true) {
        led1 = !led1;
        switch(State) {
			case 0: // INIT:
				imu.pc.printf("in INIT of case\n");
				break;
			case 1: // STANDBY:
				imu.pc.printf("in STANBY of case\n");
				break;
			case 2: // AUTO:
				if (lastState!=2) {
					for (int i=0;i<4;i++) Qset[i]=Qcur[i];
					lastState=2;
				}
				print_q("Qcur", Qcur); 
				print_q("Qset", Qset);
				imu.pc.printf("\n");
				break;
				
			case 3: //DODGE:
				imu.pc.printf("in DODGE of case\n");
			default:
			
				imu.pc.printf("in default of case, state=%d\n", State);	
		}
        //print_q("Qcur",Qcur);
        
        Thread::wait(500);
	//	get_imu();
//		float ypr[3];
//	float values[9];
//    imu.getValues(values);;
//    imu.pc.printf("ACC: %04.0f %04.0f %04.0f GYR: %04.0f %04.0f %04.0f MAG: %04.0f %04.0f %04.0f \n\r",values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8] );
//    imu.getYawPitchRoll(ypr);
//    imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f \n",ypr[0],ypr[1],ypr[2]);
		wait(0.01);
    }
}


/**
 * TCL Services
 * Larry Littlefield  larryl@tclscom
 * autopilot
 */ 
#include "mbed.h"
#include "math.h"
#include "rtos.h"
#include "IMU10DOF.h"
//#include "vector_math.h"
#include "helper_3dmath.h"

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
Quaternion Qset;  // set point for desired attitude
Quaternion Qerr;  // error values - diff fron  desired - actual
Quaternion Qcur;  // current orientation

int State=0; //  {INIT=0 ,STANBY=1, AUTO=2, DODGE=3};
int lastState=0;
float ypr_set[3], ypr_cur[3], ypr_err[3];
float ypr[3];
float values[11];  //0-2 acc, 3-5 gyro, 6-8 magn, 9-10 baro

int var2=0;
AnalogIn pin3(PA_0);
  
void pressed()  // User button oressed interupt
{
    lastState=State;
    State++;
    State = (State % 5);
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
	
  while (1) {	
    imu.getValues(values);
		if(Debug_pt==5) imu.pc.printf("ACC: %04.0f %04.0f %04.0f GYR: %04.0f %04.0f %04.0f MAG: %04.0f %04.0f %04.0f \n\r",values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8] );
    imu.getYawPitchRoll(ypr);
		if(Debug_pt==0 && State==0) imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f \n",ypr[0],ypr[1],ypr[2]);
    imu.getQ(q); 
		Qcur.w = q[0];Qcur.x = q[1]; Qcur.y = q[2]; Qcur.z = q[3];
			 // Qcur(q[0],q[1],q[2],q[3]); //for (int i=0;i<4;i++) Qcur[i]=q[i];
			if(Debug_pt==0 && State==4) {
				imu.pc.printf("q=%+5.3f %+5.3f %+5.3f %+5.3f  ", q[0], q[1], q[2], q[3]);
				imu.pc.printf("Qcur=%+5.3f %+5.3f %+5.3f %+5.3f\n", Qcur.w, Qcur.x, Qcur.y, Qcur.z);
			}
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

float rawHeading()
{	// heading = arctan(Y/X)
	float heading = atan2f(values[7], values[6]); 
	if(heading < 0.0) { heading += PI2; }
    if(heading > PI2) { heading -= PI2; }
    
	return heading*180/M_PI;
}

void print_v(float * v)
{
	imu.pc.printf("  = %+5.3f %+5.3f %+5.3f ", v[0], v[1], v[2]);
}
	
void print_q(char * Name, Quaternion q) // print a quaternion
{
	imu.pc.printf(" %s= %+5.3f %+5.3f %+5.3f %+5.3f ", Name, q.w, q.x, q.y, q.z);
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
        float hr, hc, hs, he;  // Tmp headingd
        switch(State) {
			case 0: // INIT:
				imu.pc.printf("in INIT of case\n");
				break;
			case 1: // STANDBY:
				imu.pc.printf("STANBY: ");
				imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f \n",ypr[0],ypr[1],ypr[2]);
				break;
			case 2: // AUTO:
				if (lastState!=2) {
					Qset=Qcur;
				}
				//print_q("Qcur", Qcur); 
				//print_q("Qset", Qset);
				//imu.pc.printf("\n");
				Qset.getYawPitchRoll(ypr_set);
				Qcur.getYawPitchRoll(ypr_cur);
				//print_v(ypr_set);
				//print_v(ypr_cur);
				hr = rawHeading();
				hc=Qcur.getHeading();
				hs=Qset.getHeading();
				he=Qerr.getHeading();
				imu.pc.printf("RawHead %+6.2f Cur.head %+6.2f Set.head %+6.2f Err.head %+6.2f ", hr, hc, hs, he);
				imu.pc.printf("\n");
				break;
			case 3: //DODGE:
				if (lastState!=3) {
					Qset=Qcur;
					Qerr=Qcur;
				}
				Qcur.getEulerRad(ypr_cur);
				Qset.getEulerRad(ypr_set);
				Qerr.getEulerRad(ypr_err);
				print_v(ypr_set);
				print_v(ypr_cur);
				print_v(ypr_err);
				imu.pc.printf("\n");
				break;
			case 6: //test
				for (int i=2;i<8;i++){
					imu.pc.printf("inv sqrt of %d is %f\n", (i*i), invSqrt((float)(i*i)));
				}
				wait(3);
				break;
			default:
			
				imu.pc.printf("in default of case, state=%d\n", State);	
		}
        //print_q("Qcur",Qcur);
        
        Thread::wait(100);
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


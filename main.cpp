//#define NEW
#ifdef NEW  //new main
#include "mbed.h"
#include "math.h"
//#include "rtos.h"
#include "IMU10DOF.h"

DigitalOut led1(LED1);
DigitalOut led2(PA_6);
//Ticker cache;
IMU10DOF imu(PB_9, PB_8);
byte s[4];


int var2=0;
AnalogIn pin3(PA_0);
   
 
 void fill_cache()
 {
	 uint16_t var2 = pin3.read_u16();;
	 printf("Var2-%d\n\r",var2);
	
 }


void get_imu()
{
	//fill_cache();
	float ypr[3];
	float values[9];
    imu.getValues(values);
    imu.pc.printf("ACC: %04.0f %04.0f %04.0f GYR: %04.0f %04.0f %04.0f MAG: %04.0f %04.0f %04.0f \n\r",values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8] );
    imu.getYawPitchRoll(ypr);
    imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f \n",ypr[0],ypr[1],ypr[2]);
    
    //Thread::wait(20);
    //imu.pc.printf("get_imu wait=%x  \n",Thread::wait(20)); 
}
 
void led2_thread(void const *args) {
    while (true) {
        printf("Led2 on\n\r");
        led2 = !led2;
      //  imu.pc.printf("led2_th wait=%x  \n",Thread::wait(1000));
    }
}
 
int main() {
	
	//Thread sensor(get_imu);
	//sensor.set_priority(osPriorityAboveNormal);
	imu.pc.printf("Ceci est un Test\r\n");
	imu.init(true);
	imu.gyro.status(s);imu.pc.printf(" GYR: %04i %04i %04i %04i \n\r",s[0],s[1],s[2],s[3]);
    imu.gyro.init();
    wait(1);
    imu.gyro.status(s);imu.pc.printf(" GYR2: %x %x %x %x \n\r",s[0],s[1],s[2],s[3]);
wait(5);
	//cache.attach(&get_imu, 0.1);
  //  Thread thread(led2_thread);
    
    while (true) {
        led1 = !led1;
   //     Thread::wait(500);
		//get_imu();
		float ypr[3];
	float values[9];
    imu.getValues(values);
    imu.pc.printf("ACC: %04.0f %04.0f %04.0f GYR: %04.0f %04.0f %04.0f MAG: %04.0f %04.0f %04.0f \n\r",values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8] );
    imu.getYawPitchRoll(ypr);
    imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f \n",ypr[0],ypr[1],ypr[2]);
		wait(0.01);
    }
}
#else // Old main
#include "mbed.h"
#include "math.h"


//#include "HMC5883L.h"
//#include "ADXL345_I2C.h"
//#include "L3G4200D.h"
//#include "BMP085.h"
#include "IMU10DOF.h"

//#include "PwmIn.h"

 /*
PwmOut rled(LED_RED);
PwmOut gled(LED_GREEN);
PwmOut bled(LED_BLUE);
 */
IMU10DOF imu(PB_9, PB_8);
float ypr[3];
float values[9];
double head;
int x[3];
byte s[4];
int main()
{
 //   rled=1;
 //   gled=1;

    imu.pc.printf("Ceci est un Test\r\n");
    imu.init(true);
    //imu.gyro.status(s);imu.pc.printf(" GYR: %04i %04i %04i %04i \n\r",s[0],s[1],s[2],s[3]);
    //imu.gyro.init();
   // wait(1);
    imu.gyro.status(s);imu.pc.printf(" GYR2: %x %x %x %x \n\r",s[0],s[1],s[2],s[3]);
    imu.pc.printf("Test passe\r\n");
    float alt=0;

    wait(3);
    imu.pc.printf("\n\r");
    
    while(1) {
        imu.getValues(values);
        imu.getEuler(ypr);
       // imu.gyro.readFin(ypr);  //imu.pc.printf(" GYR: %04i %04i %04i \n\r",x[0],x[1],x[2]);

        imu.pc.printf("YAW: %+5.2f PITCH: %+5.2f ROLL: %+5.2f ",ypr[0],ypr[1],ypr[2]);
        alt=imu.getBaroAlt();
        imu.pc.printf("ALT : %03.0f ",alt);
		head = imu.magn.getHeadingXY(); imu.pc.printf("HDG: %+4.1f ",(head * 180/M_PI));
		//imu.magn.getXYZ
       // imu.pc.printf("ACC: %04.0f %04.0f %04.0f GYR: %04.0f %04.0f %04.0f MAG: %04.0f %04.0f %04.0f \n\r",values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8] );
        wait_ms(1);
   //     bled=!bled;

        imu.pc.printf("\n\r");

    }





}
#endif

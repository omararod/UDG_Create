#include"Robot.h"
#include <time.h>
#include <stdlib.h>
#define BUMPER 7
#define ANGLE 20
#define DEFAULT_DELAY 5000
#define WAIT_45_DEGREESP() robot.sensors(ANGLE);while((aux+=robot.sensors(ANGLE))<37){usleep(DEFAULT_DELAY);}aux=0;
#define WAIT_45_DEGREESN() robot.sensors(ANGLE);while((aux+=robot.sensors(ANGLE))>-37){usleep(DEFAULT_DELAY);}aux=0;
#define RANDOM_IS_EVEN (rand()%2)==0
enum {NO_CRASH,CRASH_RIGHT,CRASH_LEFT,CRASH_FRONT};
char exitKey = 0;
int SPEED = 50;
void getKey() {scanf("%c",&exitKey);}
int main(int nargs, char** args)
{sleep(0);
 int aux=0;
	srand(time(NULL));
	if(nargs == 2)
	SPEED = atoi(args[1]);
	Robot robot;
	robot.safe();
	sleep(1);
	std::thread t(getKey);int i; 
	while(exitKey == 0)
	{	
		usleep(DEFAULT_DELAY);
		switch(robot.sensors(BUMPER) & 0X03)
		{
			case NO_CRASH://cout<<"Going straight"<<endl;
			robot.driveDirect(SPEED,SPEED);//Go straight
			break;
			case CRASH_RIGHT: //cout<<"Turning anti-clockwise"<<endl;
			robot.driveDirect(SPEED,-SPEED);//Dodge obstacle by the left
			WAIT_45_DEGREESP();		//Wait 45° anti-clockwise
			break;
			case CRASH_LEFT://cout<<"Turning clockwise"<<endl;
			robot.driveDirect(-SPEED,SPEED);//Dodge by the right
			WAIT_45_DEGREESN();		//Wait 45° clockwise
			break;
			case CRASH_FRONT:// cout<<"Turning randomly"<<endl;
			if(RANDOM_IS_EVEN)		//Dodge obstacle randomly
			{
				robot.driveDirect(SPEED,-SPEED);
				WAIT_45_DEGREESP();
			}		
			else
			{
				robot.driveDirect(-SPEED,SPEED);
				WAIT_45_DEGREESN();
			}
			break;
			default:
			break;
		}	
	}
	robot.driveDirect(0,0);
	t.join();
	return 0;
}

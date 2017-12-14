/*****
 * sparki express
 * sparki mail system.
 * takes orders and picks up mail and takes it to a second point
 * code for robot 1
 */

#include <Sparki.h> // include the sparki library
 
#define START 0
#define SPOT2 1
#define SPOT3 2
#define SPOT4 3
#define SPOT1 4

const int threshold = 500; // line sensors thereshold
const int objectDistance = 3; // distance to the object where the gripper has to grip it [cm]
const int objectSize = 5; // diamter of the (cylindrical) objects
const int maxObjects = 2; // number of objects to retrieve
 
const int gripTime = 1500; // time that takes to the gripper effectively grip the object [milliseconds]
const int turnSpeed = 0; //used to turn the robot over an external center of rotation.
 
int ping = 0;
int objectIndex = 0;
String state = "undefined";

//bluetooth
int commands[2];
bool gotmail = false;

//dij code 
int grid[4][4]={  
   {0, 0, 0, 0} , /*  initializers for row indexed by 0 */
   {0, 0, 0, 0} , /*  initializers for row indexed by 1 */
   {0, 0, 0, 0} , /*  initializers for row indexed by 2 */
   {0, 0, 0, 0}   /*  initializers for row indexed by 3 */
};

int dist[16];
int go_to[16];

int pos = 0; // robot position given as index on the graph
int goal= 13;// desired position
int isDone = 0;
int next;
int nextSave;
//movement functions and variables
float maxspeed=0.0285;    // [m/s] speed of the robot that you measured
float alength=0.0851;     // [m] axle length  
float phildotr=0, phirdotr=0; // wheel speeds that you sent to the motors
float Xi=0.05, Yi=0.05, Thetai=0; // where the robot is in the world
float Xrdot, Thetardot;    // how fast the robot moves in its coordinate system

float Xg;     // Where the robot should go
float Yg;
float Thetag= (3*PI)/2;

float alpha, rho, eta; // error between positions in terms of angle to the goal, distance to the goal, and final angle
float a=0.1, b=1, c=0.01; // controller gains

byte opArray[10];
int freepos = 0;
int started = 0;
int moveSet = 0;

void setup()
{
  Serial1.begin(9600);
  sparki.servo(SERVO_CENTER); // rotate the servo to is 0 degree postion (forward)
  dij(15,goal,dist); // calculate a map to go to node "13"

  //find xy of start position  
  Xi=itox(pos);
  Yi=itoy(pos);
  //find xy of goal
  Xg=itox(go_to[pos]);
  Yg=itoy(go_to[pos]);
  
  sparki.clearLCD();
  sparki.println("Starting..");
  sparki.updateLCD();
}

void opToCord(int op){
  switch (opArray[op]){
    case SPOT1:
      commands[0] = 5;
     break;
    case SPOT2:
      commands[0] = 13;
     break;
    case SPOT3:
      commands[0] = 7;
     break;
    case SPOT4:
      commands[0] = 15;
     break;
  }
  switch (opArray[op+1]){
    case SPOT1:
      commands[1] = 5;
     break;
    case SPOT2:
      commands[1] = 13;
     break;
    case SPOT3:
      commands[1] = 7;
     break;
    case SPOT4:
      commands[1] = 15;
     break;
  }
}
void loop()
{
  if (Serial1.available()) 
  {
    if (started != 1){
      byte opcode = Serial1.read();
      sparki.updateLCD();
      switch (opcode){
        case START:
          started = 1;
          sparki.print(started);
         break;
        default:
          if (freepos < 10){
            opArray[freepos] = opcode;
            sparki.print(opArray[freepos]);
            sparki.print("-");
            freepos += 1;
          }
      }
    }else{
      opToCord(moveSet);
      moveSet += 2;
      doMove();
      sparki.updateLCD();
    }
  }
}

int cost(int i,int j){
  int x1 = i / 4;
  int y1 = i % 4;
  int x2 = j / 4;
  int y2 = j % 4;

  if(i==j) return 0;
  
  if(grid[x1][y1] || grid[x2][y2]) // if obstacle
   return 99;

  int cost = fabs(x1-x2)+fabs(y1-y2);

  if(cost>1) 
   return 99;
  else
   return cost;
  
}

void dij(int n,int v,int dist[])
{
 int i,u,count,w,flag[16],min;

 for(i=0;i<=n;i++)
  flag[i]=0, dist[i]=99; // initialize every node with cost-to-goal (infinity if not adjacent)
 dist[goal]=0;
 count=1;
 while(count<=n)
 {
  min=99;
  for(w=0;w<=n;w++)                       // find node u with the current lowest score
   if(dist[w]<min && !flag[w])
    min=dist[w],u=w;
  flag[u]=1; // u is the node with the lowest score
    count++;
  for(w=0;w<=n;w++)
   if((dist[u]+cost(u,w)<dist[w]) && !flag[w]){
    dist[w]=dist[u]+cost(u,w);
    go_to[w]=u; // store where to go when at w in order to reach goal with lowest cost
   }
 }
}

// go from index in 4X4 coordinates to x-y real world
float itox(int i)
{
    int x1 = i % 4;
     float x2= .07+x1*.14;
    return x2;
}

float itoy(int i)
{
    int y1 = i / 4;
    float y2 = -.05-y1*.10;
    return y2;
}

int index(float x, float y)
{
    x=x/.14;
    x =abs(x);
    y=y/.10;
    y= abs(y);
      return x+y;
}


void readSensors()
{
  //each sensor is 1 if reading white, and 0 if reading black:
  ping = sparki.ping();
}

void gripObject()
{
  sparki.moveStop();
  state = "gripping object";
  sparki.gripperClose();
  delay(gripTime);
  sparki.gripperStop();
}
 
void releaseObject()
{
  sparki.moveStop();
  state = "releasing object";
  sparki.gripperOpen();
  delay(gripTime);
  sparki.gripperStop();
}
 
void movement() {
 rho   = sqrt((Xi-Xg)*(Xi-Xg)+(Yi-Yg)*(Yi-Yg));
      //alpha = Thetai-atan2(Yi-Yg,Xi-Xg)-PI/2.0;
      alpha = atan2(Yg-Yi,Xg-Xi)-Thetai;  
      eta   = Thetai-Thetag;

      // CALCULATE SPEED IN ROBOT COORDINATE SYSTEM
      Xrdot = a*rho;
      //Xrdot=0;
      Thetardot = b*alpha+c*eta;
  
      // CALCULATE WHEEL SPEED
      phildotr = (2*Xrdot - Thetardot*alength)/(2.0);
      phirdotr = (2*Xrdot + Thetardot*alength)/(2.0);
  
      // SET WHEELSPEED

  if(phildotr>maxspeed){
   phildotr=maxspeed;
  }
  else if(phildotr<-maxspeed){
    phildotr=-maxspeed;
  }
  if(phirdotr>maxspeed){
    phirdotr=maxspeed;
  } else if(phirdotr<-maxspeed){
    phirdotr=-maxspeed;
  }

      float leftspeed = abs(phildotr);
      float rightspeed = abs(phirdotr);

      if(leftspeed > maxspeed)
      {
        leftspeed = maxspeed;
      }
      if(rightspeed > maxspeed)
      {
        rightspeed = maxspeed;
      }
      leftspeed = (leftspeed/maxspeed)*100;//100
      rightspeed = (rightspeed/maxspeed)*100;//100

      if(rho > 0.01 || rho < -0.01)  // if farther away than 1cm
      {
        if(phildotr > 0)
        {
          sparki.motorRotate(MOTOR_LEFT, DIR_CCW,leftspeed);
        }
        else
        {
        sparki.motorRotate(MOTOR_LEFT, DIR_CW,leftspeed);
        }
        if(phirdotr > 0)
        {
          sparki.motorRotate(MOTOR_RIGHT, DIR_CW,rightspeed);
        }
        else
        {
          sparki.motorRotate(MOTOR_RIGHT, DIR_CCW,rightspeed);
        }
      }
      else
      {
        nextSave = next;
        next = go_to[pos];
        pos = nextSave;
        Xg = itox(next);
        Yg = itoy(next);
      }

      // perform odometry
      Xrdot=phildotr/2.0+phirdotr/2.0;
      Thetardot=phirdotr/alength-phildotr/alength;
      Xi=Xi+cos(Thetai)*Xrdot*0.1;
      Yi=Yi+sin(Thetai)*Xrdot*0.1;
      Thetai=Thetai+Thetardot*0.1;
}

void finished()
{
  //tells the user that the work is done:
  state = "finished";
  sparki.gripperStop();
  sparki.moveStop();
  //showSensorsAndState();
}

void doMove() {
  long int time_start = millis();
  readSensors();
  if(gotmail){
    goal = commands[1];
  }
  else
  {
    goal = commands[0];
  }
  
   dij(15,goal,dist); // calculate a map to go to goal
  
    if (pos == commands[1]){
    isDone = 1;
  }
  
  if (ping < (objectDistance * 3))
  {
    // aproximates to the detected object, but always centered on the black line
    movement();
    if ((ping <= objectDistance) && (!gotmail)) // if the object is so close, stop the robot
    {
      gripObject();
      gotmail = true;
      movement();
    }
  }
  else
  {
    movement();
    state ="moving";
  }

  if(isDone)
  {
    releaseObject();
    objectIndex++;
    if(objectIndex == maxObjects)
    {
    finished();
    }
  }

   while(millis()<time_start+100); // wait until 100ms have elapsed
}


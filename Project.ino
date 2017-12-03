#include <Sparki.h> // include the sparki library
 
String inputString; //make an empty String called inputString
boolean returnFlag; //flag to check for carriage return
char commArray [10]; //array to store communication
int arrayCounter = 0; //integer to count through commArray
 
void setup()
{
 Serial1.begin(9600);
}
 
void makeMove(){
 for(int i = 0; i <= 9; i++) 
 {
 if(commArray[i] == 'f' || commArray[i] == 'F')
 {
    sparki.moveForward();
    delay(1000); 
    sparki.moveStop();
 }
 else if (commArray[i] == 'r' || commArray[i] == 'R')
 {
    sparki.moveRight(90);
 }
 else if (commArray[i] == 'l' || commArray[i] == 'L')
 {
    sparki.moveLeft(90);
 }
 else if (commArray[i] == 's' || commArray[i] == 'S')
 {
    sparki.moveStop();
    delay(1000);
 }
}
}
 
void readComm()
{
 while (Serial1.available())
 {
    int inByte = Serial1.read();
    if ((char)inByte == 'n')
    {
      returnFlag = true;
      arrayCounter = 0;
    }
    else
    { 
 //here is where the code differs a lot from the previous code
    if(inByte == 32) //if it's a blank space
    {
      arrayCounter ++; //increment array counter to store in new array space
    }
    else
    {
 //add the character to the arrayCounter space in commArray
 // commArray[arrayCounter] += (char)inByte; //this line changed to
    commArray[arrayCounter] = (char)inByte;
    }
    }
 }
}

void instructions()
{
 //start code for reading in communication
 while (Serial1.available())
 {
    if (returnFlag)
    {
      inputString = "";
      returnFlag = false;
    }
    int inByte = Serial1.read();
    if ((char)inByte == 'n')
    {
      returnFlag = true;
    }
    else
    {
      inputString += (char)inByte; //add the character from bluetooth to string
    }
 }
 //end code for reading in communication
 
 //start code to check for OK communication
 sparki.clearLCD();
 sparki.println(inputString);
 sparki.updateLCD();
 //end code to check for OK communication
 
 Serial1.println("Sparki expects to receive communication that are");
 Serial1.println("characters for movement commands separated by spaces."); 
 Serial1.println("(F for forward, R for Right, L for Left, S for Stop)");
 Serial1.println("The line below is an example of valid communication.");
 Serial1.println("F R F L F S");
 Serial1.println("Type OK and press Send or Return to continue.");
 Serial1.println();//empty line to separate messages
 delay(2000);
 //end code for instructions
}

void loop()
{
 instructions();
 readComm();
 makeMove();
}

#include <Arduino.h>

long millisTime;
////// Gantry ///////
    int gantryColor;
    float L1;
    float L2;
    float posX;
    float posY;
    float targetL1;
    float targetL2;
    float newL1;
    float newL2;
    float t;
    float pointX, pointY;
    float dm1,dm2;
    bool m1Dir = 0;  // 0 = reel in // 1 = reel out
    bool m2Dir = 0;
    float delayM1;
    float delayM2;
    int lineLength; 

    ///// Polarbot functions //////

 

   

int bufferSize = 50;
char buffer[50];


/// Motors ////

int stepper = 200; // nema 17 1.8 degree 200 steps per revolution
int spooldia = 12; // spool diameter in mm
int feedrate = 1000; // 2000 mm/min = ~33mm/s
int frMMs = feedrate / 60;
int segmentLength = 50;
int segmentCount = 0;
float segmentRemaining = 0;
int gantryInitialLength = 260; 
int motorDistance = 950;
float spoolCir; // circumference of the spool in mm
float stepToMM;
int stepDelay;
float time;
int stepsM1;
int stepsM2;



////// CNC Shield Pins ///////
int spindle_dir = 13;
int spindle_enable = 12;
int step_enable = 8;
int dirXpin = 5;
int dirYpin = 6;
int dirZpin = 7;
int pulseXpin = 2;
int pulseYpin = 3;
int pulseZpin = 4;

//// Move motors /////

void stepM1(int dir){
    if(dir == 1){
        digitalWrite(dirXpin,HIGH);
    }else{
        digitalWrite(dirXpin,LOW);
    }
    digitalWrite(pulseXpin,HIGH);
    
    digitalWrite(pulseXpin,LOW);
}

void stepM2(int dir){
    if(dir == 1){
        digitalWrite(dirYpin,HIGH);
    }else{
        digitalWrite(dirYpin,LOW);
    }
    digitalWrite(pulseYpin,HIGH);

    digitalWrite(pulseYpin,LOW);
}

//--------------------------------------------------------------
// Forward Kinematics Turn L1 and L1 to coordinates x, ,y
// use law of cosines: theta = acos((a*a+b*b-c*c)/(2*a*b));
// to find angle between M1M2 and M1P where P is the plotter position.

void FK(float L1, float L2, float &x, float &y){
    float a = (float)L1; //* stepToMM;
    float b = motorDistance;
    float c = (float)L2; //* stepToMM;
    float theta = ((a*a+b*b-c*c)/(2.0*a*b));
    x = theta * a;
    int limit_top = 0;
    y = abs(limit_top - (sqrt( 1.0 - theta * theta ) * a));
   // Serial.print("x = "); Serial.print(x); Serial.print("  y = "); Serial.println(y);
}

//--------------------------------------------------------------

float computeL1(float x,float y){
    float length = x * x + y * y;
    return sqrt(length);
}
//--------------------------------------------------------------
float computeL2(float x,float y){
    x = motorDistance - x;
    float length = x * x + y * y;
    return sqrt(length);
}
//--------------------------------------------------------------



//--------------------------------------------------------------
//Turn XY coordinates to string length L1 and L2
void IK(float x, float y, float &L1, float &L2){
	L1 = computeL1(x,y);
	L2 = computeL2(x,y);
	
}


//--------------------------------------------------------------
void FindDistanceToPoint(float posX, float posY, float newX, float newY){
	targetL1 = 0;
	targetL2 = 0;
	//Use IK to find the length depending on the coordinates.
	IK(newX, newY, targetL1, targetL2);
	IK(posX,posY,L1,L2);
	// find the difference in current vs target length of strings
    dm1 = targetL1 - L1;
    dm2 = targetL2 - L2;
    Serial.print("Dm1 = ");Serial.print(dm1);Serial.print("   Dm2 = ");Serial.println(dm2);
	}


//-------------------------------------------------------------

void checkMotorDirection(float dm1, float dm2){
	if(dm1 > 0){
	    m1Dir = 1;
	}else{
	    m1Dir = 0;
	}
	if(dm2 > 0){
	    m2Dir = 1;
	}else{
	    m2Dir = 0;
	}
	
}
//-------------------------------------------------------------

void calculateTime(float dm1, float dm2){
	// calculate how many steps each motor must do to arrive to destination
	stepsM1 = abs(dm1) / stepToMM;
	stepsM2 = abs(dm2) / stepToMM;
    Serial.print("stepsM1 = ");Serial.print(stepsM1);Serial.print("    stepsM2 = ");Serial.println(stepsM2);
	//Now we ca find the delay for each motor
	//First we have to find the time needed for the motor that is the farthest based on our maximum speed 
	if(abs(dm1) > abs(dm2)){
	 	time = abs(dm1) / frMMs;
	 }else{
	 	time = abs(dm2) / frMMs;
	 }
	//Now we know how many steps we have to do and in how many seconds we have to do them so we can calculate the delay.
	delayM1 = abs((time * 1000) / stepsM1);
	delayM2 = abs((time * 1000) / stepsM2);
    Serial.print("delayM1 = ");Serial.print(delayM1);Serial.print("    delayM2 = ");Serial.println(delayM2);

}




//--------------------------------------------------------------
void drawLine(float x, float y){
	float currentTime = millis();
	float timeOfArrival = currentTime + (time * 1000);
    Serial.print("Estimated Time to arrival = "); Serial.println(timeOfArrival-currentTime);
	float timerM1 =  currentTime + delayM1 ;
	float timerM2 =  currentTime + delayM2 ;

    Serial.print("timerM1 = ");Serial.print(timerM1);Serial.print("    timerM2 = ");Serial.println(timerM2);
   
	while(timeOfArrival >= millis()){
		float now = millis();
		if(now >= timerM1){
			if(m1Dir == 0){
				L1 -= stepToMM;
                stepM1(0);
			}else{
				L1 += stepToMM;
                stepM1(1);
			}
			timerM1 = now + delayM1;
			FK(L1,L2,posX,posY);
		}
		if(now >= timerM2){
			if(m2Dir == 0){
				L2 -= stepToMM;
                stepM2(0);
			}else{
				L2 += stepToMM;
                stepM2(1);
			}
			timerM2 = now + delayM2;
			FK(L1,L2,posX,posY);
		} 

	}
	//FK(L1,L2,posX,posY);	
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void breakLineToSegments(float posX, float posY, float newX, float newY){
	
	int x = abs( (newX - posX) * (newX - posX) );
	int y = abs( (newY - posY) * (newY - posY) );
	lineLength = sqrt( x + y ); 
        // divide the line length with the segmentLength to see how many semgents we nee to draw 
	

	
	//find how many segments we need to draw
	segmentCount = lineLength / segmentLength;
	segmentRemaining = lineLength % segmentLength;

    
    
    if(lineLength > segmentLength){
        
        for(int i = 0; i <= segmentCount; i++){
            
            //find the ratio of line and segment
            t = float(segmentLength) / float(lineLength);
            pointX, pointY = 0;
            pointX = (1-t)*posX + t * newX;
            pointY = (1-t)*posY + t * newY;
            
            FindDistanceToPoint(posX, posY, pointX, pointY);
            checkMotorDirection(dm1,dm2);
            calculateTime(dm1, dm2);
            drawLine(pointX, pointY);
            posX = pointX;
            posY = pointY;
        }
        // if(abs(posX - newX) > 2 || abs(posY - newY) > 2){
        //     breakLineToSegments(posX,posY,newX,newY);
        // }
       /*
        FindDistanceToPoint(posX, posY, newX, newY);
        checkMotorDirection(dm1,dm2);
        calculateTime2(dm1, dm2);
        drawLine(newX, newY);
       */

        
    }else{
        FindDistanceToPoint(posX, posY, newX, newY);
        checkMotorDirection(dm1,dm2);
        calculateTime(dm1, dm2);
        drawLine(newX, newY);
    
    }
    
    
}
//////////////////////////////////

void clearBuffer(){
    for(int i = 0; i < bufferSize; i++){
        buffer[i] = NULL;
    }
}

////////////////////////////////
void setupSteppers(){
    pinMode(spindle_dir, OUTPUT);
    pinMode(spindle_enable, OUTPUT);
    pinMode(step_enable, OUTPUT);
    pinMode(dirXpin, OUTPUT);
    pinMode(dirYpin, OUTPUT);
    pinMode(pulseXpin, OUTPUT);
    pinMode(pulseYpin, OUTPUT);
    pinMode(pulseZpin, OUTPUT);

    digitalWrite(step_enable, LOW);
    Serial.println("Motors Active");

}

void initMotors(){
    // put your setup code here, to run once:
    
    spoolCir = spooldia * PI; // circumference of the spool in mm
    stepToMM = spoolCir / stepper;
    /// Find step delay based on feedrate 
    // Calculate how many steps we have to do to achieve that feedrate
    // 33mm/s   means  in 1000ms   move 33mm  
    // One step will move the thread by stemToMM so 33mm / stepToMM  = stepDelay

    stepDelay = frMMs / stepToMM;
     ////// PolarBot ///////////
    Serial.print("spoolCir =  ");Serial.println(spoolCir);
    Serial.print("stepToMM =  ");Serial.println(stepToMM);
    Serial.print("stepDelay =  ");Serial.println(stepDelay);
   
}

///////// Buffer Gcode Interprenter  /////////////


bool buffReceived = 0;
int lineCount = 0;
int xCordSerial[5];
int yCordSerial[5];

void checkBuffer(char buffer[]){

   for(int i = 0; i < bufferSize; i++){
       char c = buffer[i];
       if(c == 'X'){
           Serial.print("X value = "); 
           while(1){
               Serial.print(buffer[i+1]);
               i++;
               if(buffer[i] == ' ' || buffer[i] == '\n'){
                   break;
               }
           }
        Serial.println();
       }


       if(c == 'Y'){
           Serial.print("Y value = "); 
           while(1){
               Serial.print(buffer[i+1]);
               i++;
               if(buffer[i] == ' ' || buffer[i] == '\n'){
                   break;
               }
           }
        Serial.println();
       }
   }

}



void setup() {

    Serial.begin(115200);
    millisTime = millis();
    initMotors();
    setupSteppers();
    L1 = computeL1(motorDistance/2,gantryInitialLength);
    L2 = computeL2(motorDistance/2,gantryInitialLength);

    Serial.print("L1 = ");Serial.print(L1);
    Serial.print("    L2 = ");Serial.println(L2);
    FK(L1,L2,posX,posY);
    Serial.print("X = ");Serial.print(posX);Serial.print("    Y = ");Serial.println(posY);

}

char input;

void loop() {
   
    // put your main code here, to run repeatedly:
    int i = 0;
        if(Serial.available() > 0){
             do{
                if(Serial.available()>0){
                input = Serial.read(); 
                buffer[i]=input;
                i++;
                }
            buffReceived = 1;
            }while(input != '\n');
            
        }
    

    if(buffReceived == 1 ){
        Serial.println("RecievedBuffer");
        buffReceived = 0;
        for(int i=0; i < bufferSize; i++){
            Serial.print(buffer[i]);
        }
        checkBuffer(buffer);
    }

clearBuffer();



    // for(int i = 0; i < bufferSize; i++){
    //  // Serial.print(buffer[i]);

    // //   if(buffer[i] == 'X'){
    // //     checkBuffer(buffer , i);
    // //   }

    //   if(buffer[i] == 'c'){
    //     Serial.println("DrawingLine");
    //     //breakLineToSegments(150,50,150,200);
    //     int tx = 30;
    //     int ty = 50;
    //     FindDistanceToPoint(475,260,475+tx,260);
    //     checkMotorDirection(dm1,dm2);
    //     calculateTime(dm1, dm2);
    //     drawLine(475+50, 260);


    //     FindDistanceToPoint(475+tx,260,475+tx,260+ty);
    //     checkMotorDirection(dm1,dm2);
    //     calculateTime(dm1, dm2);
    //     drawLine(475+tx, 260+ty);


    //     FindDistanceToPoint(475+tx,260+ty,475,260+ty);
    //     checkMotorDirection(dm1,dm2);
    //     calculateTime(dm1, dm2);
    //     drawLine(475-tx, 260+ty);


    //     FindDistanceToPoint(475,260+ty,475,260);
    //     checkMotorDirection(dm1,dm2);
    //     calculateTime(dm1, dm2);
    //     drawLine(150, 50);

    //   }


    // }

   
  
}


#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    
    
    ////// FBO buffer ////////
    fbo.allocate(ofGetWidth(),ofGetHeight());
    fbo.begin();
    ofClear(255,255,255);
    fbo.end();
    
    ///////// GUI /////////////
    
    gui.setup(); // most of the time you don't need a name
    pageSetup.setup( "Board/Page Settings" );
    pageSetup.add(boardWidth.setup("Board Width",300,100,1000));
    pageSetup.add(boardHeight.setup("Board Height",500,100,1000));
    pageSetup.add(canvasYpos.setup("CanvasYpos",30,30,300));
    pageSetup.add(canvasWidth.setup("Canvas Width",300,10,500));
    pageSetup.add(canvasHeight.setup("Canvas Height",150,10,500));
    gui.add(&pageSetup);
    
    pageSetup.add(color.setup("color", ofColor(100, 100, 140), ofColor(0, 0), ofColor(255, 255)));
    
    machineGroup.setup( "Motor Settings" );
    machineGroup.add(motorSteps.setup("MotSteps", 200, 0, 1600));
    machineGroup.add(microStepping.setup("microStepping", 8, 0, 16));
    machineGroup.add(spooldia.setup( "GantryRadius" , 10, 1, 30) );
    machineGroup.add(feedrateGUI.setup("FeedrateMM/s", 30,10,60));
    machineGroup.add(segmentLength.setup("SegmentLength", 10,5,50));
    
    gui.add(&machineGroup);
    
    
    // Show GUi, collapse all tabs and load previous settings
    bHide = false;
    gui.minimizeAll();
    gui.loadFromFile( "settings.xml");
    
    //////////////////////////////////////////////////////
    gcodeCurrentLine = 0;
    
    scaleToFitScreen = 1;
    boardOfsetFromTop = 50;
    
    stepper = motorSteps; // nema 17 1.8 degree 200 steps per revolution
    spoolCir = spooldia * PI; // circumference of the spool in mm
    stepToMM = spoolCir / stepper;
    
    frMMs = feedrateGUI ; //frMMs = feedrateGUI / 60;
    segmentLength = 10;
    segmentCount = 0;
    segmentRemaining = 0;
    
    /// Find step delay based on feedrate
    // Calculate how many steps we have to do to achieve that feedrate
    // 33mm/s   means  in 1000ms   move 33mm
    // One step will move the thread by stemToMM so 33mm / stepToMM  = stepDelay
    
    stepDelay = frMMs / stepToMM;
    
    cout << "spooldia " << spooldia << endl;
    cout << "spoolCir " << spoolCir << endl;
    cout << "stepToMM " << stepToMM << endl;
    cout << "feedrate " << feedrateGUI << endl;
    cout << "stepDelay " << stepDelay << endl;
    
    
    ////// PolarBot ///////////
    
    motorDistance = ofGetWidth();
    gantryInitialLength = canvasYpos;
    
    L1 = computeL1(motorDistance/2,gantryInitialLength);
    L2 = computeL2(motorDistance/2,gantryInitialLength);
    FK(L1,L2,posX,posY);
    cout << "Current X = " << posX << endl;
    cout << "Current Y = " << posY << endl;
    
    //image.setup();
    
    millisTime = ofGetElapsedTimeMillis();
    
    ofSetVerticalSync(true);
    
    ////// Gantry  ///////////
    
    
   
    
    cout << "posX = " << posX << endl;
    cout << "posY = " << posY << endl;
    
   
    
    //serial.listDevices();
    vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
    int baud = 115200;
    //int baud = 250000;
    serial.setup(0,baud);
    //ofSetFrameRate(300);
    
}

//--------------------------------------------------------------
void ofApp::exit(){
    
}

//--------------------------------------------------------------
void ofApp::update(){
    readSerial();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    //    cam.begin();
    //    ofScale(1, -1, 1);
    //    cam.setPosition(300, -300, 500);
    //    cam.disableMouseInput();
    
    drawBoard();
    
    
    //    cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'h'){
        bHide = !bHide;
    }
    else if(key == 's'){
        gui.saveToFile("settings.xml");
    }
    else if(key == 'l'){
        gui.loadFromFile("settings.xml");
    }
    else if(key == ' '){
        color = ofColor(255);
    }
    if(key == 't'){
        millisTime = ofGetElapsedTimeMillis();
        cout << millisTime << endl;
    }
    if(key == 'b'){
        // FindTimeToPoint(300,500);
        // Teleport(300,500);
    }
    if(key == 'S'){
        char m_Test[50];
        sprintf(m_Test, "S%d %d %d %d %d %d \n", (int)motorSteps * (int)microStepping, (int)spooldia, (int)feedrateGUI, (int)segmentLength, (int)canvasYpos,  (int)boardWidth);
        serial.writeBytes(&m_Test[0], 50);
        cout << m_Test << endl;
        
    }
    if(key == 'G'){
        char m_Test[20];
//        sprintf(m_Test, "X%d Y%d Z0 \n",475,400);
//        serial.writeBytes(&m_Test[0], 20);
//
//        sprintf(m_Test, "X%d Y%d Z0 \n",400,400);
//        serial.writeBytes(&m_Test[0], 20);
//
//        sprintf(m_Test, "X%d Y%d Z0 \n",400,300);
//        serial.writeBytes(&m_Test[0], 20);
//
//
//        sprintf(m_Test, "X%d Y%d Z0 \n", 475, 300);
//        serial.writeBytes(&m_Test[0], 20);
        
        sprintf(m_Test, "X%d Y%d Z0 \n",500,300);
        serial.writeBytes(&m_Test[0], 20);
        
        sprintf(m_Test, "X%d Y%d Z0 \n",500,320);
        serial.writeBytes(&m_Test[0], 20);
        
        sprintf(m_Test, "X%d Y%d Z0 \n", 475, 320);
        serial.writeBytes(&m_Test[0], 20);
        
        sprintf(m_Test, "X%d Y%d Z0 \n",475,360);
        serial.writeBytes(&m_Test[0], 20);
        
        sprintf(m_Test, "X%d Y%d Z0 \n", 520, 360);
        serial.writeBytes(&m_Test[0], 20);
        
        sprintf(m_Test, "X%d Y%d Z0 \n", 475, 300);
        serial.writeBytes(&m_Test[0], 20);
        
    }
    if(key == 'L'){
        unsigned char buf[2] = {'L','\n'};
        serial.writeBytes(&buf[0], 2);
    }
    
    if(key == 'M'){
        //readGcode();
    }
    
    if(key == 'n'){
        launchGcode = 1;
        char m_Test[20];
        sprintf(m_Test, "X%d Y%d Z0 \n", 475, 300);
        serial.writeBytes(&m_Test[0], 20);
        
    }
}


//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
    int mouseX = ofGetMouseX();
    int mouseY = ofGetMouseY();
    
    if(mouseX > canvasXpos && mouseX < canvasXpos + canvasWidth && mouseY > canvasYpos && mouseY < canvasYpos + canvasHeight){
        cout << "Mouse X: " << mouseX << endl;
        cout << "Mouse Y: " << mouseY << endl;
        breakLineToSegments(posX,posY,mouseX,mouseY);
                char m_Test[20];
        
                sprintf(m_Test, "X%d Y%d Z0 \n", (int)mouseX - 26, (int)mouseY);
        
                serial.writeBytes(&m_Test[0], 20);

    }
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    // screenSize = ofToString(w) + "x" + ofToString(h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

//--------------------------------------------------------------
float ofApp::computeL1(float x,float y){
    float length = x * x + y * y;
    //cout << "L1 = " << sqrt(length) << endl;
    return sqrt(length);
}

//--------------------------------------------------------------
float ofApp::computeL2(float x,float y){
    x = ofGetWidth() - x;
    float length = x * x + y * y;
    //cout << "L2 = " << sqrt(length) << endl;
    return sqrt(length);
}

//--------------------------------------------------------------
void ofApp::gotoPoint(float $newL1, float $newL2,float x, float y){
    newL1 = computeL1(x,y);
    newL2 = computeL2(x, y);
    cout << "Old L1 = " << L1 << "  New L1 = " << newL1 << endl;
    cout << "Old L2 = " << L2 << "  New L2 = " << newL2 << endl;
}

//--------------------------------------------------------------
void pauseDelay(int seconds){
    int currentTime = ofGetElapsedTimeMillis();
    int destination = currentTime + seconds;
    while(destination < ofGetElapsedTimeMillis()){
        cout << "here" << endl;
//        if(currentTime + seconds > ofGetElapsedTimeMillis()){
//            break;
//        }
    }
}

//--------------------------------------------------------------
void ofApp::line(float newX, float newY){
    int currentTime = ofGetElapsedTimeMillis();
    int timeOfArrival = currentTime + (time * 1000); // time is the estimated draw time
    cout << "currentTime = " << currentTime << "   timeOfArrival = " << timeOfArrival << endl;
    cout << "Time estimate to destination = " << timeOfArrival - currentTime << endl;
    /// now we have the two delays for each motor we need to pulse them until we reach timeOfArrival
    float timerM1 = currentTime + delayM1;
    float timerM2 = currentTime + delayM2;
    
    
    while(timeOfArrival > ofGetElapsedTimeMillis()){
        float now = ofGetElapsedTimeMillis();
        if(now >= timerM1 && delayM1 > 20){
            //cout << "M1 step" << endl;
            timerM1 = now + delayM2;
            if(m1Dir == 0){
                L1 -= 2;
            }
            if(m1Dir == 1){
                L1 += 2;
            }
        }
        if(now >= timerM2 && delayM2 > 20){
            //cout << "M2 step" << endl;
            timerM2 = now + delayM1;
            if(m2Dir == 0){
                L2 -= 2;
            }
            if(m2Dir == 1){
                L2 += 2;
            }
        }
    }
    cout << "L1 = " << L1 << "  L2 = " << L2 << "  newL1 = " << newL1 << "  newL2 = " << newL2 << endl;
    FK(L1,L2,posX,posY);
}

//--------------------------------------------------------------
void ofApp::drawLine(float x, float y){
    float currentTime = ofGetElapsedTimeMillis();
    float timeOfArrival = currentTime + (time * 1000);
    float timerM1 = currentTime + delayM1;
    float timerM2 = currentTime + delayM2;
    //cout << "CurrentTime = " << currentTime << "  timeOfArrival = " << timeOfArrival << endl;
    //cout << "Estimated drawing time = " << timeOfArrival - currentTime << endl;
    //cout << "TimerM1 = " << timerM1 << "  timerM2 = " << timerM2 << endl;
    while(timeOfArrival >= ofGetElapsedTimeMillis()){
        float now = ofGetElapsedTimeMillis();
        if(now >= timerM1){
            if(m1Dir == 0){
                L1 -= stepToMM;
            }else{
                L1 += stepToMM;
            }
            timerM1 = now + delayM1;
            fbo.begin();
            FK(L1,L2,posX,posY);
            ofDrawCircle(posX,posY,0.5);
            fbo.end();
        }
        if(now >= timerM2){
            if(m2Dir == 0){
                L2 -= stepToMM;
            }else{
                L2 += stepToMM;
            }
            timerM2 = now + delayM2;
            fbo.begin();
            FK(L1,L2,posX,posY);
            ofDrawCircle(posX,posY,0.5);
            fbo.end();
            
        }
        
    }
    FK(L1,L2,posX,posY);
}

//--------------------------------------------------------------
void ofApp::Teleport(float x, float y){
    //Move the virtual gantry to position x,y
    IK(x,y,L1,L2);
}

//--------------------------------------------------------------
//Turn XY coordinates to string length L1 and L2

void ofApp::IK(float x, float y, float &L1, float &L2){
    L1 = computeL1(x,y);
    L2 = computeL2(x,y);
    
}

//--------------------------------------------------------------
// Forward Kinematics Turn L1 and L1 to coordinates x, ,y
// use law of cosines: theta = acos((a*a+b*b-c*c)/(2*a*b));
// to find angle between M1M2 and M1P where P is the plotter position.

void ofApp::FK(float L1, float L2, float &x, float &y){
    float a = (float)L1; //* stepToMM;
    float b = motorDistance;
    float c = (float)L2; //* stepToMM;
    float theta = ((a*a+b*b-c*c)/(2.0*a*b));
    //    cout << theta << endl;
    x = theta * a;
    int limit_top = 0;
    y = abs(limit_top - (sqrt( 1.0 - theta * theta ) * a));
    // cout << "L1 " << L1 << "   L2  " << L2 << endl;
    // cout << "Forward Kinematics" << endl;
    // cout << "  x =  " << x << endl;
    // cout << "  y =  " << y << endl;
}

//--------------------------------------------------------------

void ofApp::FindTimeToPoint(float posX, float posY, float newX, float newY){
    targetL1 = 0;
    targetL2 = 0;
    //cout << "Newx " << newX << "  newY  " << newY << endl;
    //Use IK to find the length depending on the coordinates.
    IK(newX, newY, targetL1, targetL2);
    IK(posX,posY,L1,L2);
    cout << "currentL1 " << L1 << "   currentL2 " << L2 <<  endl;
    cout << "targetL1 " << targetL1  <<  "   targetL2 " << targetL2  <<  endl;
    // find the difference in current vs target length of strings
    dm1 = targetL1 - L1;
    dm2 = targetL2 - L2;
    //cout << "Difference in string m1  " << dm1 << endl;
    //cout << "Difference in string m2  " << dm2 << endl;
    
}

//--------------------------------------------------------------
/*
 First see which motor has to move more and calculate how long it will take to reach destination using max speed, then calculate the delay we will have to use for the other motor in order that they will both arrive to the destination at the same time */

void ofApp::calculateTime2(float dm1, float dm2){
    // calculate how many steps each motor must do to arrive to destination
    float stepsM1 = abs(dm1) / stepToMM;
    float stepsM2 = abs(dm2) / stepToMM;
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
    //cout << "dm1 = " << dm1 << "  dm2 = " << dm2 << endl;
    //cout << "Steps m1 = " << stepsM1 << "  Steps m2 = " << stepsM2 << endl;
    //cout << "Delay m1 = " << delayM1 << "  Delay m2 = " << delayM2 << endl;
}


//-------------------------------------------------------------
void ofApp::checkMotorDirection(float dm1, float dm2){
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
    
    // cout << "m1Dir  " << m1Dir << "  m2Dir  " << m2Dir << endl;
}

//-------------------------------------------------------------
void ofApp::breakLineToSegments(float posX, float posY, float newX, float newY){
    
    int x = abs( (newX - posX) * (newX - posX) );
    int y = abs( (newY - posY) * (newY - posY) );
    lineLength = sqrt( x + y );
    // divide the line length with the segmentLength to see how many semgents we nee to draw
    
    cout << "LineLength = " << lineLength << endl;
    
    //find how many segments we need to draw
    segmentCount = lineLength / segmentLength;
    segmentRemaining = lineLength % segmentLength;
    
    cout << "SegmentCount = " << segmentCount << "  remainingSegment = " << segmentRemaining << endl;
    
    
    if(lineLength > segmentLength){
        
        for(int i = 0; i <= segmentCount; i++){
            
            //find the ratio of line and segment
            t = float(segmentLength) / float(lineLength);
            pointX, pointY = 0;
            pointX = (1-t)*posX + t * newX;
            pointY = (1-t)*posY + t * newY;
            cout << "Segment X = " << pointX << "  Segment Y = " << pointY << endl;
            
            FindTimeToPoint(posX, posY, pointX, pointY);
            checkMotorDirection(dm1,dm2);
            calculateTime2(dm1, dm2);
            drawLine(pointX, pointY);
            posX = pointX;
            posY = pointY;
        }
        if(abs(posX - newX) > 2 || abs(posY - newY) > 2){
            breakLineToSegments(posX,posY,newX,newY);
        }
        /*
         FindTimeToPoint(posX, posY, newX, newY);
         checkMotorDirection(dm1,dm2);
         calculateTime2(dm1, dm2);
         drawLine(newX, newY);
         */
        
        
    }else{
        FindTimeToPoint(posX, posY, newX, newY);
        checkMotorDirection(dm1,dm2);
        calculateTime2(dm1, dm2);
        drawLine(newX, newY);
        
    }
    
    
}

//--------------------------------------------------------------
void ofApp::drawBoard(){
    
    ofBackgroundGradient(ofColor::white, ofColor::gray);
    ofSetColor(color);
    canvasXpos = (ofGetWidth() - canvasWidth)/2 ;
    boardXpos = ofGetWidth()/2 - boardWidth/2 ;
    
    if(scaleToFitScreen == 2){
        ofDrawRectangle(boardXpos + boardWidth / 4, boardOfsetFromTop, boardWidth / scaleToFitScreen , boardHeight / scaleToFitScreen );
        ofSetColor(255);
        ofDrawRectangle(canvasXpos + canvasWidth / 4 ,canvasYpos / scaleToFitScreen,canvasWidth / scaleToFitScreen, canvasHeight / scaleToFitScreen);
    }
    if(scaleToFitScreen == 1){
        ofDrawRectangle(boardXpos, boardOfsetFromTop, boardWidth / scaleToFitScreen , boardHeight / scaleToFitScreen );
        ofSetColor(255);
        ofDrawRectangle(canvasXpos,canvasYpos / scaleToFitScreen,canvasWidth / scaleToFitScreen, canvasHeight / scaleToFitScreen);
    }
    
    
    
    
    
    
    ofSetColor(gantryColor);
    // auto draw?
    // should the gui control hiding?
    if(!bHide){
        gui.draw();
    }
    
    ofSetColor( ofColor(0, 0, ofRandom( 128, 255 ) ));
    
    fbo.draw(0,0);
    if(scaleToFitScreen == 2){
        ofDrawLine(boardXpos + boardWidth / 4, boardOfsetFromTop, posX, posY);
        ofDrawLine(boardXpos + boardWidth / 4 +  boardWidth / scaleToFitScreen, boardOfsetFromTop, posX, posY);
        ofDrawCircle(300,500,1);
    }
    if(scaleToFitScreen == 1){
        ofDrawLine(boardXpos , boardOfsetFromTop, posX, posY);
        ofDrawLine(boardXpos + boardWidth, boardOfsetFromTop, posX, posY);
        ofDrawCircle(300,500,1);
    }
    
    
}

//--------------------------------------------------------------
void ofApp::readSerial(){
    char c;
    if(serial.available()>0){
        c = (char)serial.readByte();
        cout << c;
    }
    if(c == '%'){
        drawGcode();
    }
    
}

//--------------------------------------------------------------
void ofApp::readGcode(){
    cout << "Reading Gcode File" << endl;
    lines = ofSplitString(ofBufferFromFile("test.gcode").getText(), "\n");
    linesLength = lines.size();
    //cout << lines.at(gcodeCurrentLine) << endl;

}

void ofApp::drawGcode(){
    
    if(gcodeRead == 0){
        readGcode();
        gcodeRead = 1;
    }
    
    if(gcodeCurrentLine >= linesLength){
        launchGcode = 0;
        gcodeCurrentLine = lineLength;
    }
    
    if(launchGcode == 1){
        
        string a = lines.at(gcodeCurrentLine);
        cout << endl << "Printing Line : = " << gcodeCurrentLine;
        int endX = a.find(' ');
        string cordXstr = a.substr(1,endX);
        int cordX = std::stoi( cordXstr );
        //cout << cordX << endl;

        // std::stoi( cordX );
        int endY = a.find(endX+2,' ');
        string cordYstr = a.substr(endX+2,endY);
        int cordY = std::stoi( cordYstr );
        //cout << cordY << endl;

        int offsetX = 475;
        int offsetY = 300;
//        int offsetX = 0;
//        int offsetY = 0;
    
        fbo.begin();
        //breakLineToSegments(posX, posY, cordX + offsetX, cordY + offsetY);
        ofDrawLine(posX, posY,  cordX + offsetX, cordY + offsetY);
        fbo.end();

        char m_Test[20];
        sprintf(m_Test, "X%d Y%d Z0 \n", cordX + offsetX - 26, cordY + offsetY);
        serial.writeBytes(&m_Test[0], 20);

        posX = cordX + offsetX;
        posY = cordY + offsetY;
        
        gcodeCurrentLine++;
        
    }
    
}








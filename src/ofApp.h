#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Image.hpp"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    
    void exit();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void FindTimeToPoint(float posX, float posY, float newX, float newY);
    void drawLine(float x0, float y0);
    void IK(float x, float y, float &L1, float &L2);
    void Teleport(float x, float y);
    void FK(float L1, float L2, float &x, float &y);
    void line(float x, float y);
    void checkMotorDirection(float dm1, float dm2);
    void calculateTime(float dm1, float dm2);
    void calculateTime2(float dm1, float dm2);
    float computeL1(float x, float y);
    float computeL2(float x, float y);
    void gotoPoint(float newL1,  float newL2,float x, float y);
    void breakLineToSegments(float posX, float posY, float newX, float newY); 
    void circleResolutionChanged(int & circleResolution);
    void ringButtonPressed();
    bool bHide;
    
    ////// GUI //////////
    
    ofxColorSlider color;
    ofxVec2Slider center;
    ofxIntSlider circleResolution;
    ofxToggle filled;
    ofxButton ringButton;
    ofxLabel screenSize;
    ofxFloatSlider canvasWidth;
    ofxFloatSlider canvasHeight;
    ofxPanel gui;

    long millisTime;
    ofFbo fbo;
    
    ////// Gantry ///////
    float gantryRadius;
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
    int segmentLength;
    int segmentCount;
    float segmentRemaining;
    ///// Polarbot functions //////
    int motorDistance;
    int gantryInitialLength;
    
    
    int stepper;  // nema 17 1.8 degree 200 steps per revolution
    float spooldia;  // spool diameter in mm
    float spoolCir; // circumference of the spool in mm
    float stepToMM;
    
    int feedrate; // 2000 mm/min = ~33mm/s
    int frMMs;
    
    float stepDelay;
    float time;
    
    
ImageClass image;
  
};


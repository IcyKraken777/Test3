#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);
// NOTICE, REORGANIZE THE CODE!!!


// Note for Coders: All intakes, positive is INTAKE/SCORE, Negative is OUTAKE
// Pnuematics are their given names... Eg. Wings are Wings 
void nineleft() {
    // declare initial conditions
    //9ball assumption LEFT
    PIDDataSet TestPara={1.5,0.1,0.22};
    PIDDataSet AngPara={1.7,0.1,0.18};
    //Collect 3 balls in L shape
    Wings.set(true);
    Lift.set(true);
    RunSecondStage(50);
    RunBottom(100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 70,20, 0.5,-29,true);
    MoveEncoderPID(TestPara, 30,16, 0.1,-29,true);
    wait(100,msec);
    //Possiable movement change over here add the extra 2 balls. 
    //Move and score like 4... May need to add a move back statment over here.
    TurnMaxTimePID(AngPara, -135,0.3, true);
    MoveEncoderPID(TestPara, 80,47, 0.5,-135,true);
    MoveEncoderPID(TestPara, 60,4, 0.1,-133,true);
    IntakeBoth(0);
    TurnMaxTimePID(AngPara, 0,0.5, true);
    MoveEncoderPID(TestPara, 80,28, 0.5,0,true);
    Wings.set(false);
    IntakeBoth(100);
    wait(900,msec);
    RunBottom(-100);
    wait(100,msec);
    RunBottom(100);
    wait(1100,msec);
    //Matchload
    Wings.set(true);
    MoveEncoderPID(TestPara, -100,15, 0.7,0,true);
    TurnMaxTimePID(AngPara, 0,0.5, true);
    Scrapper.set(true);
    TurnMaxTimePID(AngPara, -184,0.3, true);
    RunBottom(100);
    MoveEncoderPID(TestPara, 60,28, 0.5,181,true);
    wait(280,msec);
    //Move back and score 3
    MoveEncoderPID(TestPara, -90,15, 0.5,180,true);
    IntakeBoth(40); 
    Scrapper.set(false);
    TurnMaxTimePID(AngPara, 0,0.8, true);
    MoveEncoderPID(TestPara, 60,10, 0.2,3,true);
    Wings.set(false);
    IntakeBoth(100);
    wait(2000,msec);
    IntakeBoth(0);
    MoveEncoderPID(TestPara, -100,25, 0.1,0,true);
    MoveEncoderPID(TestPara, 100,25, 0.1,0,false);
    //Gets back into supply so we can immediately start scoring
}
void nineright(){
    //9 Ball Right (Get the OG to work first)
    PIDDataSet TestPara={1.6,0.1,0.2};
    PIDDataSet AngPara={1.7,0.1,0.18};
    Wings.set(false);
    //Tilt.set(true);
    RunBottom(100);
    //intake the 3
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 65,24, 0.5,28,true);
    MoveEncoderPID(TestPara, 30,15, 0.5,30,true);
    
}
void AWP(){
    // declare initial conditions
    //AWP
    //Solo AWP
    PIDDataSet TestPara={2.9,0.15,0.25};
    PIDDataSet AngPara={2.7,0.05,0.25};
    //Collect 3&score
    RunSecondStage(-50);
    RunBottom(100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 100,19, 0.5,-31,true);
    MoveEncoderPID(TestPara, 50,13, 0.5,-29,true);
    //RunRoller(90);
    TurnMaxTimePID(AngPara, 43,0.3, true);
    MoveEncoderPID(TestPara, 90,13, 0.5,45,true);
    IntakeBoth(-100);
    wait(2100,msec);
    IntakeBoth(0);
    MoveEncoderPID(TestPara, -80,13, 0.5,43,true);
    //Collect other 3&score
    RunBottom(100);
    //TurnMaxTimePID(AngPara, 90,0.3, true);
    MoveEncoderPID(TestPara, 100,50, 0.4,86,true);
    MoveEncoderPID(TestPara, 80,17, 0.5,87,true);
    TurnMaxTimePID(AngPara, -47,0.3, true);
    MoveEncoderPID(TestPara, 100,20, 0.3,-53,true);
    RunBottom(-80);
    //MoveEncoderPID(TestPara, 100,6, 0.2,-53,true);
    wait(200,msec);
    //MoveEncoderPID(TestPara, 100,11, 0.5,-55,true);
    //Collect match loader&score
    /*(TestPara, -100,67, 0.5,-45,true);
    Tilt.set(true);
    Clamp.set(true);
    TurnMaxTimePID(AngPara, 180,0.4, true);
    RunRoller(100);
    MoveEncoderPID(TestPara, 80,36, 0.3,180,false);
    wait(200,msec);
    MoveEncoderPID(TestPara, -100,15, 0.5,180,true);
    Tilt.set(false);
    IntakeBoth(-60); 
    TurnMaxTimePID(AngPara, 0,0.4, true);
    MoveEncoderPID(TestPara, 100,13, 0.3,-2,true);
    IntakeBoth(-100);
    //
*/
    
}
void testskills(){
    //Skills
}
//Ignore all code after this point for now
void test4(){
    //Possible 6+3
}
void test5(){
// Use this to tune PID values

}


void ninetest() {
    }

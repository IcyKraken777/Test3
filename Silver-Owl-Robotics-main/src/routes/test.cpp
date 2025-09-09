#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void test() {
    // declare initial conditions
    //7ball
    PIDDataSet TestPara={2.6,0.15,0.25};
    PIDDataSet AngPara={2.3,0.1,0.25};
    //Collect 3 balls in L shape
    Clamp.set(true);
    RunSecondStage(-50);
    RunRoller(100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 70,20, 0.5,-32,true);
    MoveEncoderPID(TestPara, 40,16, 0.5,-30,true);
    //Move and score like 4...
    TurnMaxTimePID(AngPara, -135,0.3, true);
    MoveEncoderPID(TestPara, 100,45, 0.5,-135,true);
    IntakeBoth(0);
    TurnMaxTimePID(AngPara, 0,0.5, true);
    MoveEncoderPID(TestPara, 100,26, 0.5,0,true);
    IntakeBoth(-100);
    wait(900,msec);
    RunRoller(-100);
    wait(100,msec);
    RunRoller(100);
    wait(1100,msec);
    //Matchload
    MoveEncoderPID(TestPara, -100,15, 0.5,0,true);
    Tilt.set(true);
    TurnMaxTimePID(AngPara, 180,1.3, true);
    RunRoller(100);
    MoveEncoderPID(TestPara, 100,26, 0.5,180,true);
    wait(200,msec);
    //Move back and score 3
    MoveEncoderPID(TestPara, -100,15, 0.5,180,true);
    IntakeBoth(-20); 
    Tilt.set(false);
    TurnMaxTimePID(AngPara, 0,1, true);
    MoveEncoderPID(TestPara, 85,19, 0.3,-5,true);
    IntakeBoth(-100);
    //Gets back into supply so we can immediately start scoring
}
void test2(){
    //ignore this for now
    PIDDataSet TestPara={2.8,0.1,0.25};
    PIDDataSet AngPara={2.7,0.05,0.3};
    Wings.set(false);
    //Tilt.set(true);
    RunRoller(100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 65,23, 0.5,-30,true);
    MoveEncoderPID(TestPara, 30,12, 0.5,-30,true);
    wait(300,msec);
    RunSecondStage(-35);
    RunRoller(20);
    //wait(300,msec);
    RunRoller(70);
    TurnMaxTimePID(AngPara, 45,0.5, true);
    MoveEncoderPID(TestPara, 70,16, 0.5,45,true);
    IntakeBoth(-100);
    wait(2000,msec);
    IntakeBoth(0);
    MoveEncoderPID(TestPara, -100,67, 0.5,50,true);
    TurnMaxTimePID(AngPara, 180,0.5, true);
    Tilt.set(true);
    Clamp.set(true);
    MoveEncoderPID(TestPara, 70,30, 0.5,180,true);
    RunRoller(100);
    wait(600,msec);
    Tilt.set(false);
    MoveEncoderPID(TestPara, -60,25, 0.5,175,true);
    IntakeBoth(-40);
    TurnMaxTimePID(AngPara, 0,0.5, true);
    MoveEncoderPID(TestPara, 100,8, 0.5,0,true);
    IntakeBoth(-100);
    wait(2000,msec);
    IntakeBoth(0);
    Wings.set(true);
}
void test3(){
    // declare initial conditions
    //7ball Right
    PIDDataSet TestPara={1.8,0.1,0.15};
    PIDDataSet AngPara={1.9,0.1,0.2};
    //Collect 3 balls in L shape
    IntakeBoth(-100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 80,19, 0.5,28,true);
    MoveEncoderPID(TestPara, 30,13, 0.5,30,true);
    //Move and score like 4...
    IntakeBoth(0);
    TurnMaxTimePID(AngPara, 135,0.3, true);
    MoveEncoderPID(TestPara, 100,34, 0.7,135,true);
    TurnMaxTimePID(AngPara, 0,0.5, true);
    MoveEncoderPID(TestPara, 100,24, 0.5,0,true);
    Wings.set(false);
    IntakeBoth(-100);
    wait(500,msec);
    RunRoller(-100);
    wait(100,msec);
    RunRoller(100);
    wait(1200,msec);
    //Matchload
    Wings.set(true);
    MoveEncoderPID(TestPara, -100,15, 0.5,5,true);
    Tilt.set(true);
    TurnMaxTimePID(AngPara, 185,1, true);
    RunRoller(100);
    MoveEncoderPID(TestPara, 100,24, 0.5,185,true);
    wait(300,msec);
    //Move back and score 3
    MoveEncoderPID(TestPara, -100,15, 0.5,185,true);
    IntakeBoth(-100); 
    Tilt.set(false);
    TurnMaxTimePID(AngPara, 0,0.4, true);
    MoveEncoderPID(TestPara, 100,16, 0.3,-5,true);
    Wings.set(false);
    wait(1400,msec);
    //Gets back into supply so we can immediately start scoring
    //MoveEncoderPID(TestPara, -100,15, 0.5,0,true);
    //Tilt.set(true);
    //TurnMaxTimePID(AngPara, 180,0.2, true);
    //IntakeBoth(-100);
    //MoveEncoderPID(TestPara, 100,24, 0.3,180,false);
}
void test4(){
    //Solo AWP
    PIDDataSet TestPara={2.9,0.15,0.25};
    PIDDataSet AngPara={2.7,0.05,0.25};
    //Collect 3&score
    RunSecondStage(-50);
    RunRoller(100);
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
    RunRoller(100);
    //TurnMaxTimePID(AngPara, 90,0.3, true);
    MoveEncoderPID(TestPara, 100,50, 0.4,86,true);
    MoveEncoderPID(TestPara, 80,17, 0.5,87,true);
    TurnMaxTimePID(AngPara, -47,0.3, true);
    MoveEncoderPID(TestPara, 100,20, 0.3,-53,true);
    RunRoller(-80);
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
    //Prog skills
    PIDDataSet TestPara={2.6,0.15,0.25};
    PIDDataSet AngPara={2.3,0.1,0.25};
    Wings.set(true);
    Clamp.set(true);
    RunRoller(100);
    MoveEncoderPID(TestPara, 40,34, 0.3,33,true);
    wait(800,msec);
    MoveEncoderPID(TestPara, 80,33, 0.3,140,true);
    MoveEncoderPID(TestPara, 80,29, 0.3,0,true);
    Wings.set(false);
    IntakeBoth(-80);
    wait(100,msec);
    MoveEncoderPID(TestPara, -80,8, 0.3,0,true);
    TurnMaxTimePID(AngPara, 180,0.4, true);
    Tilt.set(true);
    MoveEncoderPID(TestPara, 60,20, 0.3,180,true);
    //Pneumatics:
    //Clamp = Second stage lift (true is up)
    //Tilt = Scrapper (true to matchload)
    //Wings = gates (true to turn it on)
    //For intaking RunRoller (Postive=intake Bottom stage)
    // For Secondstage (negative = score top stage)
    //IntakeBoth (negative = scoreboth);




}

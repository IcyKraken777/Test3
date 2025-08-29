#include "../movement.hpp"
#include "../helper_functions.hpp"
#include "vex.h"
//PID Straight and turn arguments:
// MoveEncoderPID(TestPara, motor speed, encoder travel distance (inches), time to full speed(sec), relative heading(to starting position), braking?)
// TurnMaxTimePID(TestPara, Desired Heading -180 to 180, time out to calculate turn, Braking?)
// MoveTimePID(TestPara, motor speed, time traveled (sec), time to full speed, heading, false);

void test() {
    // declare initial conditions
    
    PIDDataSet TestPara={2.8,0.1,0.25};
    PIDDataSet AngPara={2.7,0.05,0.3};
    Wings.set(false);
    //Tilt.set(true);
    RunRoller(100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 70,23, 0.5,-30,true);
    MoveEncoderPID(TestPara, 30,10, 0.5,-30,true);
    RunSecondStage(-35);
    RunRoller(70);
    TurnMaxTimePID(AngPara, -135,0.25, true);
    MoveEncoderPID(TestPara, 90,47, 0.5,-135,true);
    TurnMaxTimePID(AngPara, 0,0.75, true);
    Clamp.set(true);
    MoveEncoderPID(TestPara, 100,32, 0.5,0,true);
    RunRoller(-70);
    wait(100,msec);
    IntakeBoth(-100);
    wait(2300,msec);
    IntakeBoth(0);
    
    Clamp.set(true);
    MoveEncoderPID(TestPara, -100,4, 0.2,0,true);
    TurnMaxTimePID(AngPara, 180,0.75, true);
    Tilt.set(true);
    RunRoller(100);
    MoveEncoderPID(TestPara, 90,25, 0.5,173,true);
    MoveEncoderPID(TestPara, 60,8, 0.1,180,true);
    wait(500,msec);
    Tilt.set(false);
    RunSecondStage(-45);
    RunRoller(100);
    MoveEncoderPID(TestPara, -60,30, 0.5,173,true);
    RunSecondStage(-45);
    RunRoller(100);
    TurnMaxTimePID(AngPara, 0,0.5, true);
    MoveEncoderPID(TestPara, 100,8, 0.5,0,true);
    IntakeBoth(-100);
    wait(2000,msec);
    IntakeBoth(0);
    MoveEncoderPID(TestPara, -100,4, 0.25,0,true);
    Wings.set(true);
    MoveEncoderPID(TestPara, 100,4, 0.25,0,true);
}
void test2(){
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
    PIDDataSet TestPara={2.8,0.1,0.25};
    PIDDataSet AngPara={2.7,0.05,0.3};
    //Tilt.set(true);
    Clamp.set(true);
    Tilt.set(true);
    RunRoller(100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 65,24, 0.5,30,true);
    MoveEncoderPID(TestPara, 30,10, 0.5,31,true);

    wait(300,msec);
    IntakeBoth(0);
    TurnMaxTimePID(AngPara, 135,0.5, true);
    MoveEncoderPID(TestPara, 100,57, 1.55,135,true);
    TurnMaxTimePID(AngPara, 0,0.75, true);
    Clamp.set(false);
    MoveEncoderPID(TestPara, 100,29, 0.5,0,true);
    IntakeBoth(-100);
    wait(3100,msec);
    RunSecondStage(-35);
    RunRoller(50);
    MoveEncoderPID(TestPara, -100,4, 0.5,0,true);
    TurnMaxTimePID(AngPara, 180,0.75, true);
    Tilt.set(false);
    MoveEncoderPID(TestPara, 90,45, 0.5,180,true);
    RunRoller(100);
    wait(1000,msec);
    Tilt.set(true);
    IntakeBoth(0);
    MoveEncoderPID(TestPara, -60,30, 0.5,-175,true);
    IntakeBoth(-50);
    TurnMaxTimePID(AngPara, 0,0.5, true);
    MoveEncoderPID(TestPara, 100,8, 0.5,0,true);
    IntakeBoth(-100);
    wait(2000,msec);
    IntakeBoth(0);

}
void test4(){
    //Solo AWP
    PIDDataSet TestPara={2.9,0.15,0.3};
    PIDDataSet AngPara={2.7,0.05,0.3};
    //Collect 3&score
    RunSecondStage(-50);
    RunRoller(100);
    //TurnMaxTimePID(AngPara, -30, 0.5, true);
    MoveEncoderPID(TestPara, 80,20, 0.7,-30,true);
    MoveEncoderPID(TestPara, 40,13, 0.5,-29,true);
    IntakeBoth(0);
    IntakeBoth(-40);
    wait(50,msec);
    RunRoller(70);
    TurnMaxTimePID(AngPara, 43,0.45, true);
    MoveEncoderPID(TestPara, 60,12, 0.5,43,true);
    IntakeBoth(-100);
    wait(2800,msec);
    IntakeBoth(0);
    MoveEncoderPID(TestPara, -80,13, 0.6,43,true);
    //Collect other 3&score
    RunRoller(100);
    MoveEncoderPID(TestPara, 800,50, 0.5,86,true);
    MoveEncoderPID(TestPara, 55,13, 0.5,87,true);
    wait(400,msec);
    MoveEncoderPID(TestPara, 100,20, 0.5,-55,true);
    MoveEncoderPID(TestPara, 100,11, 0.5,-55,true);
    RunRoller(-90);
    MoveEncoderPID(TestPara, 100,6, 0.5,-55,true);
    wait(200,msec);
    //Collect match loader&score
    MoveEncoderPID(TestPara, -100,65, 0.5,-45,true);
    Tilt.set(true);
    Clamp.set(true);
    TurnMaxTimePID(AngPara, 180,0.5, true);
    RunRoller(100);
    MoveEncoderPID(TestPara, 100,30, 0.5,180,true);
    wait(600,msec);
    MoveEncoderPID(TestPara, -100,15, 0.5,182,true);
    Tilt.set(false);
    IntakeBoth(50);
    wait(100,msec);
    IntakeBoth(-40); 
    TurnMaxTimePID(AngPara, 0,0.5, true);
    MoveEncoderPID(TestPara, 100,14, 0.5,0,true);
    IntakeBoth(-100);
    //

    //

}
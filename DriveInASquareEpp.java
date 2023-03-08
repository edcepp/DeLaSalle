// Autonomous program that drives bot in the shape of a square.
// For De La Salle - Method parameters specify each motor's behavior.
//
// Ed C. Epp
// January, 11 2023

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Drive in a square 1/11/23", group="Concept")
//@Disabled

// --------------------------- DriveInASquare class -----------------------------
// ------------------------------------------------------------------------------
public  class DriveInASquareEpp extends LinearOpMode
{
    double TARGET_DISTANCE    = 900;         // mm (about 18 inches)
    // This is a quess because the wheels may slip
    double DEGREES_90         = 125;         // mm wheel distance guess
    double MAX_MOTOR_VELOCITY = 600;         // mm / second
    double TARGET_VELOCITY    = MAX_MOTOR_VELOCITY / 4;
    int    SQUARE_SIDES       =   4;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Configure the motors
        leftMotor = hardwareMap.get(DcMotorEx.class,"myLeftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor = hardwareMap.get(DcMotorEx.class,"myRightMotor");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        waitForStart();

        // loop while opmode is active and
        // left or right motor are busy running to position.
        int count = 0;
        while (opModeIsActive() && count < SQUARE_SIDES)
        {
            powerUpMotors(TARGET_DISTANCE, TARGET_VELOCITY,
                          TARGET_DISTANCE, TARGET_VELOCITY);
            powerUpMotors(DEGREES_90, TARGET_VELOCITY,
                          -DEGREES_90, -TARGET_VELOCITY);
            count++;
        }


        // wait 5 sec to you can observe the final encoder position.
        Thread.sleep(5000);
    }

    // ---------- powerUpMotors -----------------------------------
    // Turn each moter a set distance and a velocity it should acheive
    //    leftMotorDistance:   target distance to move that wheel in mm
    //    velocityMm:          target velocity for that wheel in mm per 
    //                              second
 
    void powerUpMotors (double leftMotorDistance, double leftMotorVelocity,
                        double rightMotorDistance, double rightMotorVelocity)
   {
        // REV-41-1300 Core Hex Motor
        // 4 Ticks per revolution at the motor
        // Gear ration: 72:1 motor revolutions per output revolution
        // 72 motor rev per output rev * 4 ticks per rev  => 288 ticks per output rev

        // REV-41-1354 90 mm traction wheel
        // 90 mm Wheel diameter
        // 90 mm * pi => 283 mm / rev

        // compute how many encoder ticks are required for a wheel to move a given 
        // distance
        double mMPerTick = 283.0 / 288.0;
        double leftTicksToMove = leftMotorDistance / mMPerTick;
        double rightTicksToMove = rightMotorDistance / mMPerTick;       
    
        // compute the number of ticks each motor encoder should count in a second
        double leftTicksPerSecond = leftMotorVelocity / mMPerTick;
        double rightTicksPerSecond = rightMotorVelocity / mMPerTick;

        // reset the motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // set the possition to which each wheel should run
        leftMotor.setTargetPosition((int)leftTicksToMove);
        rightMotor.setTargetPosition((int)rightTicksToMove);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);    
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the velocity of each wheel
        leftMotor.setVelocity(leftTicksPerSecond);
        rightMotor.setVelocity(rightTicksPerSecond);
 
        // do nothing until each wheel has completed it mission
        while (leftMotor.isBusy() || rightMotor.isBusy())  // fixed error
        {
            telemetry.addData("position  velocity ", 
               leftMotor.getCurrentPosition() + "   " + leftMotor.getVelocity());
            telemetry.update();
            idle();
        }
        
        // each motor will stop when it is done
    }
}

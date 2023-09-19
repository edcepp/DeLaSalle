/* ***************************************************************************************
 * Copyright (c) 2021 Edward C. Epp. All rights reserved.
 *
 * Ed C. Epp 9-2021
 * MotorGotoPosition
 *
 * This is an enhancement to the TestMotor.java program.
 * It turns on the left and right motors and
 * travels for a set distance.
 * It uses velocity instead of power to turn on the motors.
 * The motor positions are displayed on the Driver Station.
 *
 * The goal is to introduce motor encoders and demonstrate the Java
 *    code the uses it.
 *   
 * Ed C. Epp 9-2023
 *    Correct statement out of order
 *    Make tweaks to the documentation standard
 ******************************************************************************************/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "motor goto position twice 9-23", group= "Concept")
// @Disabled

public class MotorGotoPositionTwice extends LinearOpMode 
{
    final    int DISTANCE = 1000;  // number of clicks to move
    final    int HALF_DISTANCE = DISTANCE / 2;
    final double VELOCITY = 300.0; // number of clicks per second
    
    DcMotorEx myLeftMotor;
    DcMotorEx myRightMotor;
    
    @Override 
    public void runOpMode () 
    {
        // Initialize the robot and wait for the user to press start
        telemetry.addData(">", " To start press play");
        telemetry.update(); 
        waitForStart();

        // Configure the left motor
        myLeftMotor  = hardwareMap.get(DcMotorEx.class, "myLeftMotor");
        myLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        myLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myLeftMotor.setTargetPosition(DISTANCE);
        myLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 
        // Configure the right motor
        myRightMotor = hardwareMap.get(DcMotorEx.class, "myRightMotor");
        myRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRightMotor.setTargetPosition(DISTANCE);
        myRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Fire up the motors
        // The order is important. 
        myLeftMotor.setVelocity(VELOCITY);
        myRightMotor.setVelocity(VELOCITY);
    
        // Let the motors run until the preset count is reached
        while (opModeIsActive() && (myLeftMotor.isBusy() || myRightMotor.isBusy()))
        {
            telemetry.addData("isBusy", myLeftMotor.isBusy());
            telemetry.addData("  left  position ", myLeftMotor.getCurrentPosition());
            telemetry.addData("  right position", myRightMotor.getCurrentPosition());
            telemetry.update();
        }
        
        myLeftMotor.setTargetPosition(HALF_DISTANCE);
        myRightMotor.setTargetPosition(HALF_DISTANCE);

        // Fire up the motors
        myLeftMotor.setVelocity(VELOCITY);
        myRightMotor.setVelocity(VELOCITY);

        // Let the motors run until the preset count is reached
        while (opModeIsActive() && (myLeftMotor.isBusy() || myRightMotor.isBusy()))
        {
            telemetry.addData("isBusy again", myLeftMotor.isBusy());
            telemetry.addData("  left  position ", myLeftMotor.getCurrentPosition());
            telemetry.addData("  right position", myRightMotor.getCurrentPosition());
            telemetry.update();
        }
 
        // Shut everything down
        myLeftMotor.setVelocity(0);
        myRightMotor.setVelocity(0);

    }
}

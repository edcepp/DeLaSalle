/* ***************************************************************************************
 * Copyright (c) 2023 Edward C. Epp. All rights reserved.
 *
 * Ed C. Epp 9-2023
 * MotorGotoPosition
 *
 * This is an enhancement to the TestMotor.java program.
 * It turns on the left and right motors and
 * travel for a set distance.
 * It uses velocity instead of power to turn on the motors.
 * The motor positions are displayed on the Driver Station.
 *
 ******************************************************************************************/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "Motor goto position 9-23", group= "Concept")
// @Disabled

public class MotorGotoPosition extends LinearOpMode 
{
    final    int DISTANCE = 1000;  // number of clicks to move
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
        myLeftMotor.setTargetPosition(DISTANCE); // do before next step
        myLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 
        // Configure the right motor
        myRightMotor = hardwareMap.get(DcMotorEx.class, "myRightMotor");
        myRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myRightMotor.setTargetPosition(DISTANCE);
        myRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Fire up the motors
        myLeftMotor.setVelocity(VELOCITY);
        myRightMotor.setVelocity(VELOCITY);
    
        // Let the motors run until the preset count is reached
        while (opModeIsActive() && (myLeftMotor.isBusy() || myRightMotor.isBusy()))
        {
            // Display position - can be removed
            telemetry.addData("  left  position ", myLeftMotor.getCurrentPosition());
            telemetry.addData("  right position", myRightMotor.getCurrentPosition());
            telemetry.update();
        }
        
        // BKM: Shut everything down
        myLeftMotor.setVelocity(0);
        myRightMotor.setVelocity(0);
    }
}

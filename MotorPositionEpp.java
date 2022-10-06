/* ***************************************************************************************
 * Copyright (c) 2021 Edward C. Epp. All rights reserved.
 *
 * Ed C. Epp 9-2021
 * Test Motors
 *
 * This is an enhancement to the TestMotorEpp.java program.
 *   It turn on the left and right motors 
 *   When the left motor reaches a present count, both
 *     motors are turned off
 *   The left motor position is displayed on the Driver Station
 *
 * The goal is to introduce motor encoders and demonstrate Java
 *    code the uses it.
 *   
 ******************************************************************************************/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous (name = "Epp motor position", group= "Concept")
// @Disabled

public class MotorPositionEpp extends LinearOpMode 
{
    @Override 
    public void runOpMode () 
    {
        // initialize the robot and wait for the user to press start
        telemetry.addData(">", " To start press play");
        telemetry.update(); 
        waitForStart();

        // fire up the motors to turn forward at 50%
        DcMotor myLeftMotor  = hardwareMap.get(DcMotorEx.class, "myLeftMotor");
        DcMotor myRightMotor = hardwareMap.get(DcMotorEx.class, "myRightMotor");
        myLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myLeftMotor.setPower(0.5);
        myRightMotor.setPower(0.5);
        
        // set left motor to run to position
        myLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        myLeftMotor.setTargetPosition(1000);
        
        
        // Let the motors run until the preset count is reached
        while (opModeIsActive() && myLeftMotor.isBusy())
        {
            telemetry.addData("position ", myLeftMotor.getCurrentPosition());
            telemetry.update();
        }
        
        // Shut everything down
        myLeftMotor.setPower(0);
        myRightMotor.setPower(0);
    }
}

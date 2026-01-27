package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous
public class RedMadisonAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor bl = null;
    private DcMotor fr = null;
    private DcMotor br = null;
    private DcMotorEx intake = null;
    private DcMotorEx outtake = null;
    private ServoImplEx hood = null;
    private ServoImplEx ur1 = null;
    private ServoImplEx ur2 = null;
    private  ServoImplEx br1 = null;
    private ServoImplEx br2 = null;
    private ServoImplEx br3 = null;
    //IMU imu;
    //int three = 2;
    //double lastbl;
    //double lastfl;
    //double lastbr;
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.6299212598; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        hood = hardwareMap.get(ServoImplEx.class, "hood");
        ur1 = hardwareMap.get(ServoImplEx.class, "ur1");
        ur2 = hardwareMap.get(ServoImplEx.class, "ur2");
        br1 = hardwareMap.get(ServoImplEx.class, "br1");
        br2 = hardwareMap.get(ServoImplEx.class, "br2");
        br3 = hardwareMap.get(ServoImplEx.class,"br3");
        waitForStart();
        telemetry.addData("frontleft", fl.getCurrentPosition());
        telemetry.addData("frontright", fr.getCurrentPosition());
        telemetry.addData("backleft", bl.getCurrentPosition());
        telemetry.addData("frontleft", encoderTicksToInches(fl.getCurrentPosition()));
        telemetry.addData("frontright", encoderTicksToInches(fr.getCurrentPosition()));
        telemetry.addData("backleft", encoderTicksToInches(bl.getCurrentPosition()));
        telemetry.update();

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hood.setDirection(Servo.Direction.FORWARD);
        ur1.setDirection(Servo.Direction.REVERSE);
        ur2.setDirection(Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //hood.setPosition(1);

        telemetry.addData("outtake", outtake.getVelocity());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Front left/Right", "%4.2f, %4.2f", flPower, frPower);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", blPower, brPower);
        telemetry.update();
        br1.setPosition(0.5);
        br2.setPosition(0.5);
        ur1.setPosition(0.5);
        ur2.setPosition(0.5);
        br3.setPosition(0.85);
        outtake.setVelocity(1200);
        hood.setPosition(.95);
        while (opModeIsActive() && runtime.seconds() < 0) {
            fl.setPower(.6);
            fr.setPower(.6);
            bl.setPower(.6);
            br.setPower(.6);
        }
        while (opModeIsActive() && runtime.seconds() < 1.3) {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        while (opModeIsActive() && runtime.seconds() <1.6) {
            //Needs +.3 Seconds
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() <2.3) {
            //Needs +.7
            br1.setPosition(.5);
            br2.setPosition(.5);
            ur1.setPosition(.5);
            ur2.setPosition(.5);
        }
        while (opModeIsActive() && runtime.seconds() <2.3) {
            // +1
            ur2.setPosition(.4);
            br3.setPosition(.44);
        }
        while (opModeIsActive() && runtime.seconds() < 3.3) {
            //+1
            ur2.setPosition(.5);
            br3.setPosition(0.85);
        }
        while (opModeIsActive() && runtime.seconds() <5.1) {
            //+1.8
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() <5.3) {
            //+.2
            br1.setPosition(.5);
            br2.setPosition(.5);
            ur1.setPosition(.5);
            ur2.setPosition(.5);
        }
        while (opModeIsActive() && runtime.seconds() <6.3) {
            //+1
            ur2.setPosition(.1);
            br3.setPosition(.44);
        }
        while (opModeIsActive() && runtime.seconds() < 7.3) {
            //+1
            ur2.setPosition(.5);
            br3.setPosition(0.85);
        }
        while (opModeIsActive() && runtime.seconds() <9.1) {
            //+1.8
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() <9.3) {
            //+.2
            br1.setPosition(.5);
            br2.setPosition(.5);
            ur1.setPosition(.5);
            ur2.setPosition(.5);
        }
        while (opModeIsActive() && runtime.seconds() <10.3) {
            //+1
            br3.setPosition(.44);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() < 11.3) {
            //+1
            br3.setPosition(0.85);
            ur2.setPosition(.5);
        }
        while (opModeIsActive() && runtime.seconds() < 12.4) {
            fl.setPower(.3);
            fr.setPower(-.3);
            bl.setPower(.3);
            br.setPower(-.3);
        }
        while (opModeIsActive() && runtime.seconds() < 14) {
            fl.setPower(.3);
            fr.setPower(.3);
            bl.setPower(.3);
            br.setPower(.3);
            intake.setVelocity(2000);
        }
        while (opModeIsActive() && runtime.seconds() < 14.8) {
            fl.setPower(0.3);
            fr.setPower(0.3);
            bl.setPower(0.3);
            br.setPower(0.3);
        }
        while (opModeIsActive() && runtime.seconds() < 15) {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        while (opModeIsActive() && runtime.seconds() <15.3) {
            //Needs +.3 Seconds
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() <16) {
            //Needs +.7
            br1.setPosition(.5);
            br2.setPosition(.5);
            ur1.setPosition(.5);
            ur2.setPosition(.5);
        }
        while (opModeIsActive() && runtime.seconds() <17) {
            // +1
            ur2.setPosition(.4);
            br3.setPosition(.44);
        }
        while (opModeIsActive() && runtime.seconds() < 18) {
            //+1
            ur2.setPosition(.5);
            br3.setPosition(0.85);
        }
        while (opModeIsActive() && runtime.seconds() <19.8) {
            //+1.8
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() <20) {
            //+.2
            br1.setPosition(.5);
            br2.setPosition(.5);
            ur1.setPosition(.5);
            ur2.setPosition(.5);
        }
        while (opModeIsActive() && runtime.seconds() <21) {
            //+1
            ur2.setPosition(.1);
            br3.setPosition(.44);
        }
        while (opModeIsActive() && runtime.seconds() < 22) {
            //+1
            ur2.setPosition(.5);
            br3.setPosition(0.85);
        }
        while (opModeIsActive() && runtime.seconds() <23.8) {
            //+1.8
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() <24) {
            //+.2
            br1.setPosition(.5);
            br2.setPosition(.5);
            ur1.setPosition(.5);
            ur2.setPosition(.5);
        }
        while (opModeIsActive() && runtime.seconds() <25) {
            //+1
            br3.setPosition(.44);
            ur2.setPosition(.1);
        }
        while (opModeIsActive() && runtime.seconds() < 26) {
            //+1
            br3.setPosition(0.85);
            ur2.setPosition(.5);
        }

        telemetry.addData("outtake velocity", outtake.getVelocity());
        telemetry.update();
        runtime.reset();


    }




}



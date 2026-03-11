package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.seattlesolvers.solverslib.controller.PIDFController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


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
public class autoBlue extends LinearOpMode {

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
    private ServoImplEx gate = null;
    public static double target = 0; //ticks
    IMU imu;
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.6299212598; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    public void telemetryGroup() {
        telemetry.addData("frontleft inches", encoderTicksToInches(fl.getCurrentPosition()));
        telemetry.addLine("the good ones");
        telemetry.addData("frontright inches", encoderTicksToInches(br.getCurrentPosition()));
        telemetry.addData("strafe inches", encoderTicksToInches(bl.getCurrentPosition()));
        telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addLine("---------");
        telemetry.addLine("others");
        telemetry.addData("outtake velocity", outtake.getVelocity());
        telemetry.addData("runtime", runtime.seconds());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
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
        imu = hardwareMap.get(IMU.class, "imu");
        gate = hardwareMap.get(ServoImplEx.class, "gate");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        telemetry.addLine("imu ready");
        telemetry.addLine("run on red side");
        telemetry.addLine("back on front of goal and tape over launch line");




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
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hood.setDirection(Servo.Direction.FORWARD);
        ur1.setDirection(Servo.Direction.REVERSE);
        ur2.setDirection(Servo.Direction.REVERSE);

        PIDFController outtakePIDF = new PIDFController(1.911,.001,.275,.7);//tuned 12-11-25 p= 1.9 i=0.001 d=0.27 f=0.7

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        //Auto Starts Here
        //front left deadwheel = fl motor
        //front right deadwheel = br motor
        //strafe deadwheel = bl motor

        gate.setPosition(0.07);
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        br1.setPosition(0.5);
        br2.setPosition(0.5);
        ur1.setPosition(0.5);
        ur2.setPosition(0.5);
        br3.setPosition(0.85);
        hood.setPosition(.7);
        double velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
        double speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
        outtake.setVelocity(speed);
        target = 1200;

        //Move forward to shoot pos
        while(encoderTicksToInches(br.getCurrentPosition())<34&&opModeIsActive()) {
            fl.setPower(.3);
            fr.setPower(.3);
            br.setPower(.3);
            bl.setPower(.3);
            telemetryGroup();
            imu.resetYaw();
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        //Shooting code
        //need to add from auto far
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .8) {
            br3.setPosition(0.85);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
            telemetryGroup();
        }
        while (opModeIsActive() && runtime.seconds() < 2) {
            br3.setPosition(0.52);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
            telemetryGroup();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .8) {
            br3.setPosition(0.85);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
            telemetryGroup();
        }
        while (opModeIsActive() && runtime.seconds() < 2) {
            br3.setPosition(0.52);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
            telemetryGroup();
        }
        while (opModeIsActive() && runtime.seconds() < 3.5) {
            br3.setPosition(0.85);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
            telemetryGroup();
        } //end shoot code

        //Turn to line up for spike
        while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)<130&&opModeIsActive()){
            fl.setPower(-.3);
            fr.setPower(.3);
            br.setPower(-.3);
            bl.setPower(.3);
            gate.setPosition(0.16);
            telemetryGroup();
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target = 0;
        }

        //Move strafe to line up with spike
        while(encoderTicksToInches(bl.getCurrentPosition())<14&&opModeIsActive()) {
            fl.setPower(.3);
            fr.setPower(-.3);
            br.setPower(-.3);
            bl.setPower(.3);
            telemetryGroup();
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            runtime.reset();
        }

        //Move to intake from spike
        while (runtime.seconds() < 1.9 && opModeIsActive()) {
            fl.setPower(.3);
            fr.setPower(.3);
            br.setPower(.3);
            bl.setPower(.3);
            intake.setVelocity(2000);
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(.1);
            telemetryGroup();
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setVelocity(0);
        //start 2nd cycle
        //drive backward to line
        while(encoderTicksToInches(br.getCurrentPosition())>-25&&opModeIsActive()) {
            fl.setPower(-.3);
            fr.setPower(-.3);
            br.setPower(-.3);
            bl.setPower(-.3);
            telemetryGroup();
            telemetry.addLine("should go back now");
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //strafe to shoot line
        while(encoderTicksToInches(bl.getCurrentPosition())>-16&&opModeIsActive()) {
            fl.setPower(-.3);
            fr.setPower(.3);
            br.setPower(.3);
            bl.setPower(-.3);
            telemetryGroup();
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            imu.resetYaw();
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        //turn to shootpos
        while(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)>-140&&opModeIsActive()){
            fl.setPower(.3);
            fr.setPower(-.3);
            br.setPower(.3);
            bl.setPower(-.3);
            telemetryGroup();
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        //shoot
        while (opModeIsActive() && runtime.seconds() <1.5) {
            //Needs +.3 Seconds
            gate.setPosition(0.16);
            //intake.setVelocity(2000); //in ticks
            br1.setPosition(.1);
            br2.setPosition(.1);
            ur1.setPosition(.1);
            ur2.setPosition(0.2);
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .8) {
        }
        while (opModeIsActive() && runtime.seconds() < 2) {
            gate.setPosition(0.2);
            br3.setPosition(0.52);
            br1.setPosition(.25);
            br2.setPosition(.25);
            ur1.setPosition(.25);
            ur2.setPosition(.4);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .8) {
            br3.setPosition(0.85);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        while (opModeIsActive() && runtime.seconds() < 2) {
            br3.setPosition(0.52);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .8) {
            br3.setPosition(0.85);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        while (opModeIsActive() && runtime.seconds() < 2) {
            br3.setPosition(0.52);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
        while (opModeIsActive() && runtime.seconds() < 8) {
            br3.setPosition(0.85);
            velocity = outtakePIDF.calculate(outtake.getVelocity(), target);
            speed = clip(velocity, 0, 2600); //may need to be higher to give more room for pidf
            outtake.setVelocity(speed);
            target = 1200;
        }
    }
}

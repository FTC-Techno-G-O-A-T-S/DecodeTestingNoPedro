/* Copyright (c) 2025 FIRST. All rights reserved.
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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
@Disabled
public class TeleopFieldCentric extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx intake = null;
    private DcMotorEx outtake = null;
    private ServoImplEx hood = null;
    private ServoImplEx ur1 = null;
    private ServoImplEx ur2 = null;
    private  ServoImplEx br1 = null;
    private ServoImplEx br2 = null;
    private ServoImplEx br3 = null;
    // This declares the four motors needed
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;


    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        hood = hardwareMap.get(ServoImplEx.class, "hood");
        ur1 = hardwareMap.get(ServoImplEx.class, "ur1");
        ur2 = hardwareMap.get(ServoImplEx.class, "ur2");
        br1 = hardwareMap.get(ServoImplEx.class, "br1");
        br2 = hardwareMap.get(ServoImplEx.class, "br2");
        br3 = hardwareMap.get(ServoImplEx.class,"br3");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hood.setDirection(Servo.Direction.FORWARD);
        ur1.setDirection(Servo.Direction.REVERSE);
        ur2.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            if (gamepad2.triangle) {
                hood.setPosition(0.75);
            } else {
                hood.setPosition(.915);
            }
            //ramp
            if (gamepad2.square) {
                ur1.setPosition(0.1);
                ur2.setPosition(0.1);
                br1.setPosition(0.1);
                br2.setPosition(0.1);
                //br3.setPosition(0.1);
            } else if (gamepad2.circle) {
                ur1.setPosition(1.0);
                ur2.setPosition(1.0);
                br1.setPosition(1.0);
                br2.setPosition(1.0);
                //br3.setPosition(1.0);
            } else {
                ur1.setPosition(0.5);
                ur2.setPosition(0.5);
                br1.setPosition(0.5);
                br2.setPosition(0.5);
                //br3.setPosition(0.5);
            }
            //kicker
            if (gamepad2.a) {
                br3.setPosition(0.35);
                //ur1.setPosition(0.1);
                //ur2.setPosition(0.1);
            } else {
                br3.setPosition(0.75);
                //ur1.setPosition(0.5);
                //ur2.setPosition(0.5);
            }
            //intake
            if (gamepad2.left_trigger > 0.4) {
                intake.setVelocity(2000); //in ticks
            } else if (gamepad2.left_bumper) {
                intake.setVelocity(-2000);
            } else {
                intake.setPower(0);
            }

            //outtake
            if (gamepad2.dpad_right) {
                outtake.setPower(0.86);
            }
            if (gamepad2.right_trigger > 0.4) {
                outtake.setVelocity(2500);
            }
            if (gamepad2.dpad_up) {
                outtake.setPower(1);
            } else {
                outtake.setVelocity(0);
            }
        }
        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        //fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private boolean opModeIsActive() {
        return false;
    }

    private void waitForStart() {
    }

    @Override
    public void loop() {
        telemetry.addLine("Restart teleop to reset Yaw");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad2.dpad_down) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }


    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        fl.setPower(maxSpeed * (frontLeftPower / maxPower));
        fr.setPower(maxSpeed * (frontRightPower / maxPower));
        bl.setPower(maxSpeed * (backLeftPower / maxPower));
        br.setPower(maxSpeed * (backRightPower / maxPower));
    }
}

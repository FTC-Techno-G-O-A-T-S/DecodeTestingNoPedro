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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PController;



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
@Config
@TeleOp(name="teleop", group="Linear OpMode")
//@Disabled
public class teleop extends LinearOpMode {

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
    private Servo light = null;
    //private org.firstinspires.ftc.teamcode.PIDFController PIDFController;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    //PIDFController pidf = new PIDFController();
    public static double output = 0;
    public static double velocity = 0;
    public static double setpoint = 0;




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
        light = hardwareMap.get(Servo.class, "light");


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
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(Servo.Direction.FORWARD);
        ur1.setDirection(Servo.Direction.REVERSE);
        ur2.setDirection(Servo.Direction.REVERSE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        //hood.setPosition(1);


        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            //hood
            if (gamepad2.triangle){
                hood.setPosition(0.75);
            } else{
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
            if(gamepad2.a){
                br3.setPosition(0.42);
                //ur1.setPosition(0.1);
                //ur2.setPosition(0.1);
            } else{
                br3.setPosition(0.76);
                //ur1.setPosition(0.5);
                //ur2.setPosition(0.5);
            }
            //intake
            if (gamepad2.left_trigger > 0.4) {
                intake.setVelocity(2000); //in ticks
            } else if (gamepad2.left_bumper){
                intake.setVelocity(-2000);
            } else {
                intake.setPower(0);
            }

            //outtake
            if (gamepad2.dpad_right) {
                outtake.setPower(0.86);
            } if (gamepad2.right_trigger > 0.4) {
                outtake.setVelocity(2500);
            } if (gamepad2.dpad_up) {
                outtake.setPower(1);
            } else {
                outtake.setVelocity(0);
            }

            if (gamepad2.options) {
                light.setPosition(0.333);
            }
            // Calculates the output of the PIDF algorithm based on sensor
            // readings. Requires both the measured value
            // and the desired setpoint
            /*output = pidf.calculate(
                //outtake.getCurrentPosition(), setpoint
            );*/

            /*
             * A sample control loop for a motor
             */
            //PController pController = new PController(kP);

        // We set the setpoint here.
        // Now we don't have to declare the setpoint
        // in our calculate() method arguments.
           // pController.setSetPoint(velocity);

        // perform the control loop
            /*
             * The loop checks to see if the controller has reached
             * the desired setpoint within a specified tolerance
             * range
             *//*
            while (!pController.atSetPoint()) {
                output = pController.calculate(
                        outtake.getVelocity()  // the measured value
                );
                outtake.setVelocity(output);
            }
            outtake.setVelocity(0); // stop the motor

        // NOTE: motors have internal PID control

*/








            //DRIVE CODE
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial;   //= -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral; //=  gamepad1.left_stick_x;
            double yaw;     //=  gamepad1.right_stick_x;
            if(gamepad1.right_trigger > 0.4) {
                axial = -gamepad1.left_stick_y * 0.40;
                lateral = gamepad1.left_stick_x * 0.40;
                yaw = gamepad1.right_stick_x * 0.40;
            } else {
                axial= -gamepad1.left_stick_y;
                lateral = gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            fl.setPower(frontLeftPower);
            fr.setPower(frontRightPower);
            bl.setPower(backLeftPower);
            br.setPower(backRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("outtake velocity", outtake.getVelocity());
            telemetry.addData("outtake power", outtake.getPower());
            telemetry.update();
        }
    }}

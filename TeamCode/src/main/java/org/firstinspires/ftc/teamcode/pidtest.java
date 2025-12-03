package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
@Disabled
public class pidftest extends LinearOpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0.19;

    public static int target = 0;

    private final double ticks_in_degree = 384.5 / 360;

    private DcMotorEx outtake;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int velocity = outtake.getVelocity();
        double pid = controller.calculate(velocity, target);
        //double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;


        double power = pid + f;

        outtake.setVelocity(power);
        //arm_motor2.setPower(power);
      //  telemetry.addData( "pos", armPos);
       // telemetry.addData( "target", target);
        telemetry.update();
    }
}

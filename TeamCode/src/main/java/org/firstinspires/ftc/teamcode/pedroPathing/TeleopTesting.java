package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.AllMechs;


import java.util.ArrayList;
import java.util.List;

@TeleOp(name="TeleOp Testing")
public class TeleopTesting extends OpMode {
    AllMechs robot;

    Gamepad currentGamepad1;
    Gamepad previousGamepad1;


    private final FtcDashboard dash = FtcDashboard.getInstance();
   @Override
    public void init() {
        // Step 1: Initialize the robot, which maps the physical motors.
        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

    }


    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.leftFront.setPower(frontLeftPower);
        robot.leftBack.setPower(backLeftPower);
        robot.rightFront.setPower(frontRightPower);
        robot.rightBack.setPower(backRightPower);


        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (gamepad1.cross) {
            CommandManager.INSTANCE.scheduleCommand(
                    robot.transfer()
            );
        }
        if(gamepad1.circle){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.turnOffFlywheel()
            );
        }
        if(gamepad1.square){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.pushTransfer()
            );
        }
        if(gamepad1.triangle){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOff()
            );
        }
        if(gamepad1.dpad_down){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.setLow()
            );
        }
        if(gamepad1.dpad_up){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.setHigh()
            );
        }
        if(gamepad1.dpad_left){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.resetAll()
            );
        }
        if(gamepad2.circle){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.setLow()
            );
        }
        if(gamepad2.square){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.setMiddle()
            );
        }



//        if (currentGamepad1.cross && !previousGamepad1.cross) {
//            intakeInToggle = !intakeInToggle;
//        }
//        if (intakeInToggle) {
//            runningCommands.add(
//                    new InstantAction(() ->
//                            robot.intake.setPower(-0.5)
//                    ));
//
//        }
//        if (currentGamepad1.triangle && !previousGamepad1.triangle) {
//            intakeOutToggle = !intakeOutToggle;
//        }
//        if (intakeOutToggle) {
//            runningCommands.add(
//                    new InstantAction(() ->
//                            robot.intake.setPower(.5)
//                    ));
//
//        }

        CommandManager.INSTANCE.run();



    }
}

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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
//    Follower follower;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;

    private final Pose startPose = new Pose(0, 0, 0);

    boolean horToggle = false;
    public static final double clawOpen = 0;
    public static final double clawClose = 1;


    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void init() {
        // Step 1: Initialize the robot, which maps the physical motors.
        robot = new AllMechs(hardwareMap, gamepad1, gamepad2);

//        follower = Constants.createFollower(hardwareMap);
//
//        follower.setStartingPose(startPose);
//        follower.update();
        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

    }

//    @Override
//    public void start() {
//        follower.startTeleopDrive();
//    }

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
//        TelemetryPacket packet = new TelemetryPacket();
//        double axial = .484038 * Math.tan(1.12 * -gamepad1.left_stick_y);
//        double lateral = .484038 * Math.tan(1.12 * -gamepad1.left_stick_x);
//        double rotational = -gamepad1.right_stick_x;
//
//        follower.setTeleOpDrive(axial, lateral, rotational, true);
//
//        follower.update();
//
//        Pose currentPose = follower.getPose();
////
//        if (currentPose != null) {
////            // Dashboard field overlay (uncomment this if you want to use the overlay)
////            // Canvas fieldOverlay = packet.fieldOverlay();
////            //
////            // double currentX = currentPose.getX();
////            // double currentY = currentPose.getY();
////            // double currentHeading = currentPose.getHeading();
////            //
////            // fieldOverlay.setStroke("red");
////            // fieldOverlay.strokeCircle(currentX, currentY, 9);
////            // fieldOverlay.strokeLine(currentX, currentY,
////            //         currentX + 9 * Math.cos(currentHeading),
////            //         currentY + 9 * Math.sin(currentHeading));
//        } else {
////            // Add a friendly message to the telemetry if the pose hasn't been initialized yet
//            telemetry.addLine("Waiting for Follower to initialize Pose...");
//        }
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (gamepad1.a) {
            CommandManager.INSTANCE.scheduleCommand(
                    robot.setFullPower()
            );
        }
        if(gamepad1.b){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.turnOff()
            );
        }
        if(gamepad1.x){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOn()
            );
        }
        if(gamepad1.y){
            CommandManager.INSTANCE.scheduleCommand(
                    robot.intakeOff()
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

//        packet.addLine("Current Robot Pose: " + follower.getPose().toString());
//        packet.addLine("Running Commands: " + CommandManager.INSTANCE.getRunningCommands().size());
//        dash.sendTelemetryPacket(packet);

    }
}

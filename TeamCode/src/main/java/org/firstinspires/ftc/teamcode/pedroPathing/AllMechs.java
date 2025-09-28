package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.CommandGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.SingleFunctionCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.PassiveConditionalCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import com.rowanmcalpin.nextftc.core.units.TimeSpan;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class AllMechs {

    public DcMotor leftFront, leftBack, rightFront, rightBack, flyWheel, intake;

    public MultipleTelemetry telemetry;
    public Gamepad gamepad1;
    public Gamepad gamepad2;


    public AllMechs(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {


        leftFront = hardwareMap.get(DcMotor.class, "front left");
        leftBack = hardwareMap.get(DcMotor.class, "back left");
        rightFront = hardwareMap.get(DcMotor.class, "front right");
        rightBack = hardwareMap.get(DcMotor.class, "back right");
        flyWheel = hardwareMap.get(DcMotor.class, "fwMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);



        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = new MultipleTelemetry();


    }


    public Command setFullPower(){
        return new InstantCommand(()-> flyWheel.setPower(1));
    }
    public Command turnOff(){
        return new InstantCommand(()-> flyWheel.setPower(0));
    }
    public Command intakeOn() {
        return new InstantCommand(()-> intake.setPower(1));
    }
    public Command intakeOff(){
        return new InstantCommand(()-> intake.setPower(0));
    }
//    public Command setVertTarget(int target) {
//        return new InstantCommand(() -> vert_target = target);
//    }
//
//    public Command updateVertPID() {
//        return new SingleFunctionCommand(() -> {
//            double rightPos = vert_right.getCurrentPosition();
//            double pid = controller_vert.getError();
//            double ff = Math.cos(Math.toRadians(vert_target / ticks_in_degree)) * f;
//            double power = pid + ff;
//            vert_left.setPower(power);
//            vert_right.setPower(power);
//            telemetry.addData("Vert Power", power);
//            telemetry.addData("Vert Position", rightPos);
//            return Math.abs(vert_target - rightPos) < 10; // Finish when within 10 ticks
//        });
//    }
////    public Command updateExtPID() {
////        return new SingleFunctionCommand(() -> {
////            int Pos = extension.getCurrentPosition();
////            double pid = controller_extension.getError();
////
////            double ff = Math.cos(Math.toRadians(hor_target / ticks_in_degree)) * f;
////
////            double power = pid + ff;
////
////
////            extension.setPower(power);
////
////            return true;
////        });
////    }
////    public Command setExtTarget(int extarget) {
////        return new InstantCommand(() -> hor_target = extarget);
////    }
//
//    public Command intakeIn(){
//        return new InstantCommand(()
//                -> intake.setPower(-0.5));
//    }
//
//    public Command intakeBack(){
//        return new InstantCommand(()
//                -> intake.setPower(0.5));
//    }
//
//
//    public Command vertSample(){
//        return new InstantCommand(()
//                -> vert_target = 2700);
//    }
//
//    public Command vertDown() {
//        return new InstantCommand(()
//                -> vert_target = 50);
//    }
//
////    public Command setHorTarget(int target) {
////        return new InstantCommand(() -> hor_target = target);
////    }
//
////    public Command horExtend() {
////        return new InstantCommand(()
////                -> hor_target = 350);
////    }
//
////    public Command horRetract() {
////        return new InstantCommand(()
////                -> hor_target = 20);
////    }
//
//
//    public Command intakeDown() {
//        return new InstantCommand(()
//                -> hold.setPosition(.75));
//    }
//
//    public Command intakeUp() {
//        return new InstantCommand(()
//                -> hold.setPosition(.3));
//    }
//
//    public Command checkColorRed() {
//        return new ParallelGroup(
//                // Step 1: Set hold servo
//                new InstantCommand(() -> hold.setPosition(0.75)),
//
//                // Step 2: Check color and perform conditional actions
//                new PassiveConditionalCommand(
//                        () -> colorSensor.red() > colorSensor.green() + 50 && colorSensor.red() > colorSensor.blue() + 50,
//                        () -> new ParallelGroup(
//                                new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
//                                new InstantCommand(() -> gamepad1.setLedColor(255, 0, 0, 5000)),
//                                new InstantCommand(() -> gamepad1.rumbleBlips(1)),
//                                new InstantCommand(() -> intake.setPower(0)),
//                                new InstantCommand(() -> hold.setPosition(0.3))
//
//                        ),
//                        () -> new PassiveConditionalCommand(
//                                () -> colorSensor.green() > colorSensor.blue() && colorSensor.red() > colorSensor.blue(),
//                                () -> new ParallelGroup(
//                                        new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
//                                        new InstantCommand(() -> gamepad1.setLedColor(230, 230, 0, 5000)),
//                                        new InstantCommand(() -> gamepad1.rumbleBlips(1)),
//                                        new InstantCommand(() -> intake.setPower(0)),
//                                        new InstantCommand(() -> hold.setPosition(0.3))
//
//                                ),
//                                () -> new PassiveConditionalCommand(
//                                        () -> colorSensor.blue() > colorSensor.green() + 50 && colorSensor.blue() > colorSensor.red() + 50,
//                                        () -> new ParallelGroup(
//                                                new InstantCommand(() -> pooper.setPosition(POOPER_OPEN)),
//                                                new InstantCommand(() -> gamepad1.setLedColor(0, 0, 255, 5000)),
//                                                new InstantCommand(() -> gamepad1.rumbleBlips(1)),
//                                                new InstantCommand(() -> intake.setPower(-0.65)),
//                                                new Delay(TimeSpan.fromMs(500))
//                                        ),
//                                        () -> new PassiveConditionalCommand(
//                                                ()-> gamepad1.square,
//                                                () -> new ParallelGroup(
//                                                        new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
//                                                        new InstantCommand(() -> hold.setPosition(0.3)),
//                                                        new InstantCommand(() -> intake.setPower(0))
//                                                ),
//                                                () -> new ParallelGroup(
//                                                        new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
//                                                        new InstantCommand(() -> intake.setPower(-0.65))
//                                                )
//                                        )
//                                )
//                        )
//                )
//        );
//    }
//    public Command stopIntake() {
//        return new ParallelGroup(
//                new InstantCommand(() -> pooper.setPosition(POOPER_BLOCK)),
//                new InstantCommand(() -> intake.setPower(0))
//        );
//    }
//
//    public Command armUp() {
//        return new SequentialGroup(
//                new InstantCommand(() -> arm_left.setPosition(arm_left_up)),
//                new InstantCommand(() -> arm_right.setPosition(arm_right_up))
//        );
//    }
//    public Command armDown() {
//        return new SequentialGroup(
//                new InstantCommand(() -> arm_left.setPosition(arm_left_down)),
//                new InstantCommand(() -> arm_right.setPosition(arm_right_down))
//        );
//    }
//    public Command wristUp() {
//        return new ParallelGroup(
//                new InstantCommand(() -> wrist_left.setPosition(wrist_left_up)),
//                new InstantCommand(() -> wrist_right.setPosition(wrist_right_up))
//        );
//    }
//    public Command wristDown() {
//        return new ParallelGroup(
//                new InstantCommand(() -> wrist_left.setPosition(wrist_left_down)),
//                new InstantCommand(() -> wrist_right.setPosition(wrist_right_down))
//        );
//    }
//
//    public Command clawClose() {
//        return new InstantCommand(() -> claw.setPosition(CLAW_CLOSE));
//
//    }
//    public Command clawOpen() {
//        return new InstantCommand(() -> claw.setPosition(CLAW_OPEN));
//
//    }
//    public Command rotateHor() {
//        return new InstantCommand(() -> rotate.setPosition(rotate_hor));
//
//    }
//    public Command rotateVert() {
//        return new InstantCommand(() -> rotate.setPosition(rotate_vert));
//
//    }
//    public Command armWait(){
//        return new ParallelGroup(
//                new InstantCommand(()-> arm_right.setPosition(arm_right_wait)),
//                new InstantCommand(()-> arm_left.setPosition(arm_left_wait))
//        );
//    }
}




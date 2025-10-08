package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Testing", group = "testing")
public class ServoPos extends OpMode {
    public Servo height_Servo;
    public CRServo pushLeft, pushRight ;
    public DcMotor flywheel;
    public static double servo_high_pos = 1;
    public static double servo_middle_pos = 0.6;
    public static double servo_low_pos = 0.2;


    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotor.class, "fwMotor");
        height_Servo = hardwareMap.get(Servo.class, "height");
        pushLeft = hardwareMap.get(CRServo.class, "pushLeft");
        pushRight = hardwareMap.get(CRServo.class, "pushRight");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            height_Servo.setPosition(servo_low_pos);
        }

        if (gamepad1.dpad_down) {
            height_Servo.setPosition(servo_high_pos);

        }
        if (gamepad1.dpad_right) {
            height_Servo.setPosition(servo_middle_pos);

        }
        if(gamepad1.dpad_left){
            flywheel.setPower(1);
        }
        if(gamepad1.square){
            pushLeft.setPower(1);
            pushRight.setPower(-1);
        }
        if(gamepad1.circle){
            pushLeft.setPower(0);
            pushRight.setPower(0);
        }


    }
}

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import kotlin.properties.ObservableProperty;

@TeleOp(name="intakeTest", group = "testing")
public class intakeTest extends OpMode {
    public DcMotor intake;


    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            intake.setPower(1);
        }
    }
}

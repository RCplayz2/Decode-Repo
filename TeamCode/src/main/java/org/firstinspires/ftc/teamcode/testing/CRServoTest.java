package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import kotlinx.coroutines.internal.OpDescriptor;

@TeleOp
public class CRServoTest extends OpMode {
    public CRServo pushLeft;
    public CRServo pushRight;



    @Override
    public void init() {
        pushLeft = hardwareMap.get(CRServo.class, "pushLeft");
        pushRight = hardwareMap.get(CRServo.class, "pushRight");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            pushLeft.setPower(1);
            pushRight.setPower(-1);
        }
    }
}

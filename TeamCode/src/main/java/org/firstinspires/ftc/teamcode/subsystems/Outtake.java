package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.CachedMotor;

import static org.firstinspires.ftc.teamcode.pedroPathing.RobotConstants.*;

public class Outtake {
    public enum RobotState {
        CLOSE,
        TAPE_CORNER,
        FAR_ZONE,
        TRANSITION

    }
    public enum OuttakePower {
        THREE_FOURTHS,
        FULL,
        OFF
    }
    public enum OuttakeAngle {
        //change names to actual values
        LOW,
        MIDDLE,
        HIGH,
        DEFAULT
    }

    public CachedMotor outtake_left, outtake_right;
    public Servo height_servo;
    public RobotState robotstate;
    public OuttakePower outtakepower = OuttakePower.OFF;
    public OuttakeAngle outtakeAngle = OuttakeAngle.DEFAULT;
    public Telemetry telemetry;

//need to add a sort of distance threshold to characterize all of these states into. Could use april tag localization or use the pinpoint localizer and create separate methods for each coord or sm.

    public Outtake(HardwareMap hardwaremap, Telemetry telemetry){

        outtake_left = new CachedMotor(hardwaremap.get(DcMotor.class, "outtake_left"));
        outtake_right = new CachedMotor(hardwaremap.get(DcMotor.class, "outtake_right"));
        height_servo = hardwaremap.get(Servo.class, "height");

        outtake_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        setRobotState(RobotState.TRANSITION);
        this.telemetry = telemetry;
    }
    //fix
    public void setRobotState(RobotState robotstate){
        this.robotstate = robotstate;

        if(robotstate == RobotState.CLOSE){
            setOuttakePower(OuttakePower.THREE_FOURTHS);
            setOuttakeAngle(OuttakeAngle.HIGH);
        } else if (robotstate == RobotState.FAR_ZONE){
            setOuttakeAngle(OuttakeAngle.LOW);
            setOuttakePower(OuttakePower.FULL);
        } else if (robotstate == RobotState.TAPE_CORNER){
            setOuttakePower(OuttakePower.THREE_FOURTHS);
            setOuttakeAngle(OuttakeAngle.MIDDLE);
        } else if (robotstate == RobotState.TRANSITION) {
            setOuttakePower(OuttakePower.OFF);
            setOuttakeAngle(OuttakeAngle.DEFAULT);
        }

    }
    public void setOuttakePower(OuttakePower outtakepower){
        this.outtakepower = outtakepower;

        if(outtakepower == OuttakePower.THREE_FOURTHS){
            outtake_left.setPower(0.7);
            outtake_right.setPower(0.7);

        } else if (outtakepower == OuttakePower.FULL){
            outtake_left.setPower(1);
            outtake_right.setPower(1);

        } else if (outtakepower == OuttakePower.OFF){
            outtake_left.setPower(0);
            outtake_right.setPower(0);

        }
    }

    public void setOuttakeAngle(OuttakeAngle outtakeAngle){
        this.outtakeAngle = outtakeAngle;

        if(outtakeAngle == OuttakeAngle.HIGH){
            height_servo.setPosition(servo_high_pos);
        } else if(outtakeAngle == OuttakeAngle.MIDDLE){
            height_servo.setPosition(servo_middle_pos);
        } else if(outtakeAngle == OuttakeAngle.LOW){
            height_servo.setPosition(servo_low_pos);
        } else if(outtakeAngle == OuttakeAngle.DEFAULT){
            height_servo.setPosition(servo_default_pos);
        }

    }

    public void telemetry() {
        if(telemetry != null) {
            telemetry.addData("Outtake State", robotstate);
            telemetry.addData("Outtake Power", outtakepower);
            telemetry.addData("Outtake Angle", outtakeAngle);
            telemetry.addData("Servo Position", height_servo.getPosition());
        }
    }

    public void periodic(){
        telemetry();
    }






}


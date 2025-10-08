package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor; // Added for setZeroPowerBehavior

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
// Removed unnecessary imports: android.graphics.Color and org.opencv.core.Scalar

public class Intake {

    public enum IntakeState {
        IDLE,       // Motor off
        INTAKING,   // Motor on, actively searching for a game element
        DETECTED,   // Motor stopped, game element successfully secured
        EJECTING    // Motor reversed to push game element out
    }

    // Hardware
    public ColorSensor colorSensor;
    public DcMotorEx intakeMotor;

    // State & Configuration
    private Telemetry telemetry; // Encapsulated telemetry
    private IntakeState intakeState;
    private final double INTAKE_POWER = 0.5; // Constant for intake speed
    private final double CURRENT_THRESHOLD = 2.0; // Amps: If current exceeds this, assume jam

    // NOTE: Raw R/G/B values are specific to your environment and sensor.
    // This is a placeholder for a 'purple' check.
    private final int COLOR_R_MIN = 100;
    private final int COLOR_B_MIN = 150;
    private final int COLOR_G_MAX = 50;


    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialization
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color sensor");
        this.telemetry = telemetry;

        // Configuration
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial state
        switchState(IntakeState.IDLE);
    }

    public void switchState(IntakeState newState) {
        this.intakeState = newState;

        if (newState == IntakeState.IDLE || newState == IntakeState.DETECTED) {
            intakeMotor.setPower(0);
        }
    }

    public IntakeState getCurrentState() {
        return intakeState;
    }

    public void periodic() {
        switch (intakeState) {

            case IDLE:
                // Motor remains off (set in switchState)
                break;

            case INTAKING:
                // 1. Set intake motor power
                intakeMotor.setPower(INTAKE_POWER);

                // 2. Check for Jam (Current Limit)
                double current = intakeMotor.getCurrent(CurrentUnit.AMPS);
                if (current > CURRENT_THRESHOLD) {
                    // You might want to briefly reverse, stop, or simply reduce power
                    // For now, we'll stop to signal a jam/stall.
                    telemetry.addData("INTAKE", "JAM DETECTED!");
                    intakeMotor.setPower(0);
                    // switchState(IntakeState.EJECTING); // A better solution might be auto-eject
                }

                // 3. Check for Game Element Detection (Color)
                // Assuming you are looking for a purple element
                int red = colorSensor.red();
                int blue = colorSensor.blue();
                int green = colorSensor.green();

                if (red >= COLOR_R_MIN && blue >= COLOR_B_MIN && green <= COLOR_G_MAX) {
                    switchState(IntakeState.DETECTED);
                }
                break;

            case DETECTED:
                // Motor is stopped (set in switchState). Ready for transfer.
                break;

            case EJECTING:
                // Run the motor in reverse
                intakeMotor.setPower(-INTAKE_POWER);
                break;
        }
    }

    // Public methods for driver/command system interface
    public void startIntake() {
        switchState(IntakeState.INTAKING);
    }

    public void stopIntake() {
        switchState(IntakeState.IDLE);
    }

    public void eject() {
        switchState(IntakeState.EJECTING);
    }

    public void telemetry() {
        if (telemetry != null) {
            telemetry.addData("Intake State", intakeState);
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Motor Current (A)", intakeMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Color (R, G, B)", String.format("%d, %d, %d", colorSensor.red(), colorSensor.green(), colorSensor.blue()));
        }
    }
}

package org.firstinspires.ftc.teamcode.pedroPathing;
// Check
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.AllMechs;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Testing Auto", group = "Examples")
public class AutoTesting extends OpMode {

    // ========== HARDWARE & SUBSYSTEMS ==========
    private Follower follower;
    private AllMechs allMechs;  // Contains all robot mechanisms

    // ========== PATH CHAINS ==========
    private PathChain path1Chain, path2Chain, path3Chain;

    // ========== TIMERS ==========
    private Timer pathTimer, actionTimer, opModeTimer, intakeDelayTimer;

    // ========== STATE MANAGEMENT ==========
    private int pathState = -1;
    private int shotsFired = 0;
    private boolean pushing = false;
    private boolean intakeStarted = false;
    private boolean transferring = false;

    // ========== TIMING CONSTANTS ==========
    // Shooting sequence timing
    private static final double SPINUP_SECONDS = 0.6;           // Flywheel spinup before first shot
    private static final double PUSH_DURATION_SEC = 0.30;       // Duration to push one ball through
    private static final double INTERSHOT_DELAY_SEC = 0.25;     // Recovery time between shots
    private static final int SHOTS_PER_POSITION = 3;            // Number of balls to shoot

    // Intake and transfer timing
    private static final double INTAKE_STARTUP_DELAY_SEC = 1.0; // REQUIRED: 1 second delay before intake starts
    private static final double TRANSFER_DURATION_SEC = 0.5;    // Time for transfer servos to move ball
    private static final double INTAKE_RUN_TIME_SEC = 3.0;      // Maximum time to run intake per cycle

    // ========== MOTOR/SERVO POWER CONSTANTS ==========
    private static final double FLYWHEEL_POWER = 1.0;           // Shooter motor power
    private static final double INTAKE_POWER = 1.0;             // Intake motor power
    private static final double PUSHER_LEFT_POWER = 1.0;        // Left transfer servo power
    private static final double PUSHER_RIGHT_POWER = -1.0;      // Right transfer servo power (opposite direction)

    // ========== SERVO POSITIONS ==========
    // Height servo positions for angle adjustment (shooting angles)
    private static final double HEIGHT_SHOOTING_HIGH = AllMechs.servo_high_pos;    // High basket shot
    private static final double HEIGHT_SHOOTING_MID = AllMechs.servo_middle_pos;   // Mid basket shot
    private static final double HEIGHT_SHOOTING_LOW = AllMechs.servo_low_pos;      // Low basket shot

    // ========== FIELD POSE CONSTANTS ==========
    private final Pose startPose = new Pose(57.083, 10.144, Math.toRadians(90));                      // Starting position
    private final Pose pickup1Pose = new Pose(42.564, 35.204, Math.toRadians(180));                  // First pickup location
    private final Pose midPose = new Pose(11.536, 35.602, Math.toRadians(180));                      // Mid-field waypoint
    private final Pose finalPose = new Pose(57.2817679558011, 9.944751381215468, Math.toRadians(120)); // Final scoring position

    // ========== INITIALIZATION ==========
    @Override
    public void init() {
        // Initialize all timers
        pathTimer = new Timer();
        actionTimer = new Timer();
        opModeTimer = new Timer();
        intakeDelayTimer = new Timer();
        opModeTimer.resetTimer();

        // Initialize follower (Pedro Pathing for autonomous navigation)
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);
            telemetry.addData("✓ Follower", "Initialized");
        } catch (Exception e) {
            telemetry.addData("✗ Follower ERROR", e.getMessage());
            follower = null;
        }

        // Initialize AllMechs (all robot hardware)
        try {
            allMechs = new AllMechs(hardwareMap, gamepad1, gamepad2);
            telemetry.addData("✓ AllMechs", "Initialized");

            // Verify critical hardware
            boolean intakeOk = (allMechs.intake != null);
            boolean shooterOk = (allMechs.flyWheel != null);
            boolean transferOk = (allMechs.pushLeft != null && allMechs.pushRight != null);
            boolean heightOk = (allMechs.height != null);

            telemetry.addData("  Intake Motor", intakeOk ? "✓" : "✗ MISSING");
            telemetry.addData("  Shooter Motor", shooterOk ? "✓" : "✗ MISSING");
            telemetry.addData("  Transfer Servos", transferOk ? "✓" : "✗ MISSING");
            telemetry.addData("  Height Servo", heightOk ? "✓" : "✗ MISSING");

            if (!intakeOk || !shooterOk || !transferOk) {
                telemetry.addData("⚠ WARNING", "Critical hardware missing!");
            }
        } catch (Exception e) {
            telemetry.addData("✗ AllMechs ERROR", e.getMessage());
            allMechs = null;
        }

        // Build all autonomous paths
        if (follower != null) {
            buildPaths();
            telemetry.addData("✓ Paths", "Built successfully");
        } else {
            telemetry.addData("✗ Paths", "Cannot build - follower failed");
        }

        telemetry.addData("═════════════", "");
        telemetry.addData("Status", "READY TO START");
        telemetry.update();
    }

    /**
     * Builds all path chains for autonomous navigation
     */
    private void buildPaths() {
        // Path 1: Start position -> First pickup location
        path1Chain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Path 2: First pickup -> Mid-field (intake and transfer happen during this path)
        path2Chain = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, midPose))
                .setTangentHeadingInterpolation()
                .build();

        // Path 3: Mid-field -> Final scoring position
        path3Chain = follower.pathBuilder()
                .addPath(new BezierLine(midPose, finalPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "⏳ Waiting for START");
        telemetry.addData("Time", "%.1f sec", opModeTimer.getElapsedTimeSeconds());

        // Update localization
        if (follower != null) {
            follower.update();
            Pose pose = follower.getPose();
            telemetry.addData("Position", "X:%.1f Y:%.1f H:%.0f°",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        }

        telemetry.update();
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(0);
        telemetry.addData("Status", "▶ AUTONOMOUS STARTED");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update localization
        if (follower != null) {
            follower.update();
        }

        // Execute state machine
        autonomousPathUpdate();

        // Display telemetry
        updateTelemetry();
    }

    @Override
    public void stop() {
        stopAllMechanisms();

        telemetry.addData("Status", "⏹ STOPPED");
        telemetry.addData("Total Runtime", "%.2f sec", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Total Shots", shotsFired);
        telemetry.update();
    }

    // ========== AUTONOMOUS STATE MACHINE ==========
    /**
     * Main autonomous sequence controller
     *
     * SEQUENCE:
     * ┌─────────────────────────────────────────────────────────────┐
     * │ PHASE 1: Shoot 3 Preloaded Balls (States 0, 10)            │
     * │   - Spin up flywheel                                        │
     * │   - Fire 3 balls with timed pusher pulses                   │
     * ├─────────────────────────────────────────────────────────────┤
     * │ PHASE 2: Navigate & Intake (States 1, 2, 20)               │
     * │   - Start path to pickup                                    │
     * │   - Wait 1 second (REQUIRED DELAY)                          │
     * │   - Start intake while continuing to move                   │
     * │   - Collect ball from field                                 │
     * ├─────────────────────────────────────────────────────────────┤
     * │ PHASE 3: Transfer While Moving (States 30, 31)             │
     * │   - Transfer ball from intake to shooter chamber            │
     * │   - Continue moving to mid-field simultaneously             │
     * ├─────────────────────────────────────────────────────────────┤
     * │ PHASE 4: Navigate to Final Position (State 4)              │
     * │   - Move to final scoring location                          │
     * ├─────────────────────────────────────────────────────────────┤
     * │ PHASE 5: Final Shooting (State 40)                         │
     * │   - Fire collected ball(s)                                  │
     * ├─────────────────────────────────────────────────────────────┤
     * │ COMPLETE (State 99)                                         │
     * └─────────────────────────────────────────────────────────────┘
     */
    private void autonomousPathUpdate() {
        switch (pathState) {

            // ═══════════════════════════════════════════════════════
            // PHASE 1: SHOOT PRELOADED BALLS
            // ═══════════════════════════════════════════════════════

            case 0:
                // Initialize shooting sequence for preloaded balls
                initializeShooting(HEIGHT_SHOOTING_HIGH);  // Set to high basket angle
                setPathState(10);
                break;

            case 10:
                // Execute shooting sequence - fire 3 preloaded balls
                if (executeShooting()) {
                    // All preloaded balls fired, begin navigation
                    telemetry.addData("Phase 1", "✓ Complete - 3 balls fired");
                    setPathState(1);
                }
                break;

            // ═══════════════════════════════════════════════════════
            // PHASE 2: NAVIGATE & INTAKE
            // ═══════════════════════════════════════════════════════

            case 1:
                // Start moving toward first pickup location
                if (follower != null) {
                    follower.followPath(path1Chain);
                    intakeDelayTimer.resetTimer();  // Start 1-second countdown
                    intakeStarted = false;
                    telemetry.addData("Phase 2", "Started - Moving to pickup");
                    setPathState(2);
                }
                break;

            case 2:
                // Moving to pickup location + waiting for 1 second intake delay
                double delayElapsed = intakeDelayTimer.getElapsedTimeSeconds();

                // Check if 1 second has passed
                if (!intakeStarted && delayElapsed >= INTAKE_STARTUP_DELAY_SEC) {
                    // Delay complete - START INTAKE
                    startIntake();
                    intakeStarted = true;
                    actionTimer.resetTimer();
                    telemetry.addData("Intake", "✓ Started (after 1s delay)");
                    setPathState(20);
                }
                // Robot continues moving during the delay period
                break;

            case 20:
                // Intake is actively running while robot moves
                double intakeRunTime = actionTimer.getElapsedTimeSeconds();

                // Determine when to stop intake (multiple exit conditions)
                boolean intakeTimeout = intakeRunTime >= INTAKE_RUN_TIME_SEC;
                boolean pathComplete = (follower != null && !follower.isBusy());

                // Optional: Could add ball detection here if you have sensors
                // boolean ballDetected = checkBallInIntake();

                if (intakeTimeout || pathComplete) {
                    // Stop intake
                    stopIntake();
                    telemetry.addData("Phase 2", "✓ Complete - Ball collected");

                    // Start path to mid-field
                    if (follower != null) {
                        follower.followPath(path2Chain, true);
                    }

                    actionTimer.resetTimer();
                    transferring = false;
                    setPathState(30);
                }
                break;

            // ═══════════════════════════════════════════════════════
            // PHASE 3: TRANSFER WHILE MOVING
            // ═══════════════════════════════════════════════════════

            case 30:
                // Transfer ball from intake chamber to shooter chamber
                // This happens WHILE robot is moving to mid-field

                if (!transferring) {
                    // Start transfer mechanism
                    startTransfer();
                    transferring = true;
                    actionTimer.resetTimer();
                    telemetry.addData("Transfer", "✓ Started (while moving)");
                }

                // Check if transfer duration is complete
                double transferElapsed = actionTimer.getElapsedTimeSeconds();
                if (transferElapsed >= TRANSFER_DURATION_SEC) {
                    // Transfer complete
                    stopTransfer();
                    telemetry.addData("Transfer", "✓ Complete");
                    setPathState(31);
                }
                break;

            case 31:
                // Continue moving to mid-field (transfer already finished)
                if (follower != null && !follower.isBusy()) {
                    // Reached mid-field, proceed to final scoring position
                    follower.followPath(path3Chain, true);
                    telemetry.addData("Phase 3", "✓ Complete - At mid-field");
                    setPathState(4);
                }
                break;

            // ═══════════════════════════════════════════════════════
            // PHASE 4: NAVIGATE TO FINAL POSITION
            // ═══════════════════════════════════════════════════════

            case 4:
                // Moving to final scoring position
                if (follower != null && !follower.isBusy()) {
                    // Arrived at final position, prepare to shoot
                    telemetry.addData("Phase 4", "✓ Complete - At final position");
                    initializeShooting(HEIGHT_SHOOTING_HIGH);
                    setPathState(40);
                }
                break;

            // ═══════════════════════════════════════════════════════
            // PHASE 5: FINAL SHOOTING
            // ═══════════════════════════════════════════════════════

            case 40:
                // Fire collected ball(s) from final position
                if (executeShooting()) {
                    // All shots complete
                    telemetry.addData("Phase 5", "✓ Complete - All balls fired");
                    setPathState(99);
                }
                break;

            case 99:
                // AUTONOMOUS COMPLETE
                stopAllMechanisms();
                telemetry.addData("═══════════", "");
                telemetry.addData("Status", "✓ AUTONOMOUS COMPLETE");
                telemetry.addData("═══════════", "");
                break;

            default:
                // SAFETY: Unknown state
                telemetry.addData("⚠ ERROR", "Unknown state: " + pathState);
                stopAllMechanisms();
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // SHOOTING MECHANISM METHODS
    // ═══════════════════════════════════════════════════════════════

    /**
     * Initialize shooting sequence
     * Spins up flywheel and sets height servo angle
     *
     * @param heightPosition - servo position for shooting angle (high/mid/low)
     */
    private void initializeShooting(double heightPosition) {
        shotsFired = 0;
        pushing = false;
        actionTimer.resetTimer();

        // Start flywheel motor (outtake/shooter motor)
        if (allMechs != null && allMechs.flyWheel != null) {
            allMechs.flyWheel.setPower(FLYWHEEL_POWER);
        }

        // Set height servo to desired shooting angle
        if (allMechs != null && allMechs.height != null) {
            allMechs.height.setPosition(heightPosition);
        }
    }

    /**
     * Execute shooting state machine
     * Manages flywheel spinup, pusher timing, and shot counting
     *
     * State flow:
     * 1. Wait for flywheel spinup (first shot only)
     * 2. Activate pusher servos
     * 3. Wait for push duration
     * 4. Deactivate pusher servos
     * 5. Wait inter-shot delay
     * 6. Repeat until all shots fired
     *
     * @return true when all shots complete, false otherwise
     */
    private boolean executeShooting() {
        double elapsed = actionTimer.getElapsedTimeSeconds();

        // Check if all shots fired
        if (shotsFired >= SHOTS_PER_POSITION) {
            // Stop flywheel
            if (allMechs != null && allMechs.flyWheel != null) {
                allMechs.flyWheel.setPower(0);
            }
            return true;  // Shooting complete
        }

        // Pusher state machine
        if (!pushing) {
            // WAITING STATE - delay before next push
            double requiredDelay = (shotsFired == 0) ? SPINUP_SECONDS : INTERSHOT_DELAY_SEC;

            if (elapsed >= requiredDelay) {
                // Time to push
                startPusher();
                pushing = true;
                actionTimer.resetTimer();
            }
        } else {
            // PUSHING STATE - pusher servos active
            if (elapsed >= PUSH_DURATION_SEC) {
                // Push complete
                stopPusher();
                pushing = false;
                shotsFired++;
                actionTimer.resetTimer();
            }
        }

        return false;  // Still shooting
    }

    /**
     * Activate pusher servos to feed ball into flywheel
     * Uses pushLeft and pushRight continuous rotation servos
     */
    private void startPusher() {
        if (allMechs != null) {
            if (allMechs.pushLeft != null) {
                allMechs.pushLeft.setPower(PUSHER_LEFT_POWER);
            }
            if (allMechs.pushRight != null) {
                allMechs.pushRight.setPower(PUSHER_RIGHT_POWER);
            }
        }
    }

    /**
     * Stop pusher servos
     */
    private void stopPusher() {
        if (allMechs != null) {
            if (allMechs.pushLeft != null) {
                allMechs.pushLeft.setPower(0);
            }
            if (allMechs.pushRight != null) {
                allMechs.pushRight.setPower(0);
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // INTAKE MECHANISM METHODS
    // ═══════════════════════════════════════════════════════════════

    /**
     * Start intake motor to collect balls from field
     */
    private void startIntake() {
        if (allMechs != null && allMechs.intake != null) {
            allMechs.intake.setPower(INTAKE_POWER);
        }
    }

    /**
     * Stop intake motor
     */
    private void stopIntake() {
        if (allMechs != null && allMechs.intake != null) {
            allMechs.intake.setPower(0);
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // TRANSFER MECHANISM METHODS
    // ═══════════════════════════════════════════════════════════════

    /**
     * Start transfer mechanism
     * Moves ball from intake chamber to shooter chamber
     * Uses ONLY pushLeft and pushRight servos (height servo NOT involved)
     */
    private void startTransfer() {
        if (allMechs != null) {
            // Activate transfer servos
            if (allMechs.pushLeft != null) {
                allMechs.pushLeft.setPower(PUSHER_LEFT_POWER);
            }
            if (allMechs.pushRight != null) {
                allMechs.pushRight.setPower(PUSHER_RIGHT_POWER);
            }
        }
    }

    /**
     * Stop transfer mechanism
     */
    private void stopTransfer() {
        if (allMechs != null) {
            // Stop transfer servos
            if (allMechs.pushLeft != null) {
                allMechs.pushLeft.setPower(0);
            }
            if (allMechs.pushRight != null) {
                allMechs.pushRight.setPower(0);
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // SAFETY & UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════

    /**
     * Emergency stop all mechanisms
     * Called during stop() or error conditions
     */
    private void stopAllMechanisms() {
        if (allMechs != null) {
            // Stop flywheel/shooter
            if (allMechs.flyWheel != null) {
                allMechs.flyWheel.setPower(0);
            }

            // Stop intake
            if (allMechs.intake != null) {
                allMechs.intake.setPower(0);
            }

            // Stop transfer servos
            if (allMechs.pushLeft != null) {
                allMechs.pushLeft.setPower(0);
            }
            if (allMechs.pushRight != null) {
                allMechs.pushRight.setPower(0);
            }
        }
    }

    /**
     * Change state and reset path timer
     */
    private void setPathState(int newState) {
        pathState = newState;
        if (pathTimer != null) {
            pathTimer.resetTimer();
        }
    }

    // ═══════════════════════════════════════════════════════════════
    // TELEMETRY METHODS
    // ═══════════════════════════════════════════════════════════════

    /**
     * Comprehensive telemetry display
     */
    private void updateTelemetry() {
        // Header
        telemetry.addData("═══════════════════", "");
        telemetry.addData("AUTONOMOUS STATUS", "");
        telemetry.addData("═══════════════════", "");

        // Current state
        telemetry.addData("State", "%d: %s", pathState, getStateDescription(pathState));
        telemetry.addData("Runtime", "%.1f sec", opModeTimer.getElapsedTimeSeconds());

        // Action status
        telemetry.addData("───────────", "");
        telemetry.addData("Shots Fired", "%d / %d", shotsFired, SHOTS_PER_POSITION);
        telemetry.addData("Pushing", pushing ? "✓ YES" : "✗ NO");
        telemetry.addData("Intake Active", intakeStarted ? "✓ YES" : "✗ NO");
        telemetry.addData("Transferring", transferring ? "✓ YES" : "✗ NO");

        // Robot position
        if (follower != null && follower.getPose() != null) {
            Pose pose = follower.getPose();
            telemetry.addData("───────────", "");
            telemetry.addData("Position", "X:%.1f Y:%.1f", pose.getX(), pose.getY());
            telemetry.addData("Heading", "%.0f°", Math.toDegrees(pose.getHeading()));
            telemetry.addData("Path Active", follower.isBusy() ? "✓ YES" : "✗ NO");
        }

        // Hardware status
        if (allMechs != null) {
            telemetry.addData("───────────", "");
            if (allMechs.flyWheel != null) {
                telemetry.addData("Shooter", "%.0f%%", allMechs.flyWheel.getPower() * 100);
            }
            if (allMechs.intake != null) {
                telemetry.addData("Intake", "%.0f%%", allMechs.intake.getPower() * 100);
            }
            if (allMechs.height != null) {
                telemetry.addData("Height Angle", "%.2f", allMechs.height.getPosition());
            }
        }

        // Special timing displays
        telemetry.addData("───────────", "");
        if (pathState == 2) {
            // Show intake delay countdown
            double remaining = INTAKE_STARTUP_DELAY_SEC - intakeDelayTimer.getElapsedTimeSeconds();
            telemetry.addData("Intake Delay", "%.2f sec remaining", Math.max(0, remaining));
        } else if (pathState == 10 || pathState == 40) {
            // Show shooting progress
            telemetry.addData("Action Timer", "%.2f sec", actionTimer.getElapsedTimeSeconds());
        } else if (pathState == 30) {
            // Show transfer progress
            double transferProgress = (actionTimer.getElapsedTimeSeconds() / TRANSFER_DURATION_SEC) * 100;
            telemetry.addData("Transfer", "%.0f%% complete", Math.min(100, transferProgress));
        }

        telemetry.update();
    }

    /**
     * Get human-readable state description
     */
    private String getStateDescription(int state) {
        switch (state) {
            case -1: return "Uninitialized";
            case 0:  return "Init Shooting";
            case 10: return "Shooting Preloaded (Phase 1)";
            case 1:  return "Path to Pickup Started";
            case 2:  return "Moving + Intake Delay (1s)";
            case 20: return "Intake Running (Phase 2)";
            case 30: return "Transferring (Phase 3)";
            case 31: return "Moving to Mid-Field";
            case 4:  return "Moving to Final (Phase 4)";
            case 40: return "Final Shooting (Phase 5)";
            case 99: return "✓ COMPLETE";
            default: return "Unknown State";
        }
    }
}
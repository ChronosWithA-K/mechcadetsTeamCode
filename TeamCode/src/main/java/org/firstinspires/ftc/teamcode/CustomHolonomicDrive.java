package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Basic: Custom Holonomic Drive", group="Linear OpMode")
public class CustomHolonomicDrive extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor viperSlideMotor = null;

//    private DcMotor xEncoder = null; // If uncommented and not attached to the robot, things will break
//    private DcMotor yEncoder = null;

    private Servo extendServo = null;
    private Servo bucketServo = null;
    private Servo intakeServo = null;
    private Servo clawServo = null;
    private Servo clawWristServo = null;

    enum State {
        IDLE,
        EXTENDED,
        GRABBED,
        LOADED,
        LIFTED,
        DROP
    }
    private State state = State.IDLE;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

//        xEncoder = hardwareMap.get(DcMotor.class, "x_encoder");
//        yEncoder = hardwareMap.get(DcMotor.class, "y_encoder");

        extendServo = hardwareMap.get(Servo.class, "extend_servo");
        bucketServo = hardwareMap.get(Servo.class, "bucket_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawWristServo = hardwareMap.get(Servo.class,  "claw_wrist_servo");

        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_slide_motor");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        viperSlideMotor.setTargetPosition(0);
        viperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlideMotor.setPower(1);

        viperSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double max;
        double extendServoPosition = 0.0;
        double bucketServoPosition = 0.0;
        double intakeServoPosition = 0.0;
        double clawServoPosition = 0.0;
        double clawWristServoPosition = 0.0; // Servo left value is too far down, centre is perfect

        int viperSlideMotorPosition = 0;

        int liftDown = 0;
        int liftUp = 3100;
        double bucketDrop = 0.37;
        double bucketLoad = 0.5;
        double extendClosed = 0;
        double extendExtended = 1;
        double intakeDown = 0.2;
        double intakeUp = 1;
        double wristLoad = 0.5;
        double wristDrop = 1;
        double clawClosed = 0;
        double clawOpen = 0.5;

        boolean prevExtend = false;
        boolean prevBucket = false;
        boolean prevIntake = false;
        boolean prevClawWrist = false;
        boolean prevClaw = false;

        boolean prevViper = false;

        boolean aPrev = false;
        boolean bPrev = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double axialThreshold = 0.05 * lateral;
            double lateralThreshold = 0.05 * axial;
            // If not steering much, assume it's because of human inaccuracy and fix it (untested)
            if (Math.abs(axial) <= axialThreshold) {
                axial = 0;
            } else if (Math.abs(lateral) <= lateralThreshold) {
                lateral = 0;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);

            boolean a = gamepad1.a && !aPrev;
            aPrev = gamepad1.a;
            boolean b = gamepad1.b && !bPrev;
            bPrev = gamepad1.b;

            switch (state) {
                case IDLE:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristDrop;
                    clawServoPosition = clawClosed;

                    if (a) {
                        state = State.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    clawWristServoPosition = wristLoad;
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.IDLE;
                    } else if (b) {
                        state = State.GRABBED;
                    }
                    break;
                case GRABBED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendExtended;
                    intakeServoPosition = intakeDown;
                    clawWristServoPosition = wristLoad;
                    clawServoPosition = clawClosed;

                    if (a) {
                        state = State.LOADED;
                    } else if (b) {
                        state = State.EXTENDED;
                    }
                    break;
                case LOADED:
                    viperSlideMotorPosition = liftDown;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristDrop;
                    clawServoPosition = clawClosed;

                    if (a) {
                        state = State.LIFTED;
                    }
                    break;
                case LIFTED:
                    viperSlideMotorPosition = liftUp;
                    bucketServoPosition = bucketLoad;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristDrop;
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.DROP;
                    }
                    break;
                case DROP:
                    viperSlideMotorPosition = liftUp;
                    bucketServoPosition = bucketDrop;
                    extendServoPosition = extendClosed;
                    intakeServoPosition = intakeUp;
                    clawWristServoPosition = wristDrop;
                    clawServoPosition = clawOpen;

                    if (a) {
                        state = State.IDLE;
                    }
                    break;
            }

//
//            // Viper slide motor logic
//            boolean viper = gamepad1.left_bumper;
//            if (!prevViper) {
//                if (viper && viperSlideMotorPosition == 0) {
//                    viperSlideMotorPosition = 3100;
//                } else if (viper && viperSlideMotorPosition == 3100) {
//                    viperSlideMotorPosition = 0;
//                }
//            }
//            prevViper = viper;
//
//            // Servo position logic
//            boolean extend = gamepad1.a;
//            if (!prevExtend) {
//                if (extend && extendServoPosition == 0.0) {
//                    extendServoPosition = 1.0;
//                } else if (extend && extendServoPosition == 1.0) {
//                    extendServoPosition = 0.0;
//                }
//            }
//            prevExtend = extend;
//
//            boolean bucket = gamepad1.b;
//            if (!prevBucket) {
//                if (bucket && bucketServoPosition == 0.0) {
//                    bucketServoPosition = 1.0;
//                } else if (bucket && bucketServoPosition == 1.0) {
//                    bucketServoPosition = 0.0;
//                }
//            }
//            prevBucket = bucket;
//
//            boolean intake = gamepad1.y;
//            if (!prevIntake) {
//                if (intake && intakeServoPosition == 0.0) {
//                    intakeServoPosition = 1.0;
//                } else if (intake && intakeServoPosition == 1.0) {
//                    intakeServoPosition = 0.0;
//                }
//            }
//            prevIntake = intake;
//
//            boolean clawWrist = gamepad1.right_bumper;
//            if (!prevClawWrist) {
//                if (clawWrist && clawWristServoPosition == 0.0) {
//                    clawWristServoPosition = 1.0;
//                } else if (clawWrist && clawWristServoPosition == 1.0) {
//                    clawWristServoPosition = 0.0;
//                }
//            }
//            prevClawWrist = clawWrist;
//
//            boolean claw = gamepad1.x;
//            if (!prevClaw) {
//                if (claw && clawServoPosition == 0.0) {
//                    clawServoPosition = 1.0;
//                } else if (claw && clawServoPosition == 1.0) {
//                    clawServoPosition = 0.0;
//                }
//            }
//            prevClaw = claw;

            // Set servo positions
            extendServo.setPosition(extendServoPosition);
            bucketServo.setPosition(bucketServoPosition);
            intakeServo.setPosition(intakeServoPosition);
            clawServo.setPosition(clawServoPosition);
            clawWristServo.setPosition(clawWristServoPosition);

             // Set (non-drive) motor power
            viperSlideMotor.setTargetPosition(viperSlideMotorPosition);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("xEncoder", "%4.2f, %4.2f", xEncoder);
//            telemetry.addData("yEncoder", "%4.2f, %4.2f", yEncoder);
            telemetry.addData("State", state);
            telemetry.addData("extendServo position: ", extendServoPosition);
            telemetry.addData("bucketServo position: ", bucketServoPosition);
            telemetry.addData("intakeServo position: ", intakeServoPosition);
            telemetry.addData("clawServo position: ", clawServoPosition);
            telemetry.addData("clawWristServo position: ", clawWristServoPosition);
            telemetry.addData("Viper encoder: ", viperSlideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

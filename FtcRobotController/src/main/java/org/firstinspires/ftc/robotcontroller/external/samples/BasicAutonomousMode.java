/*
 * First Autonomous code file
 */

 package org.firstinspires.ftc.robotcontroller.external.samples;

 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 
 /*
  * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
  * Each motion axis is controlled by one Joystick axis.
  *
  * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
  * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
  * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
  *
  * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
  * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
  * the direction of all 4 motors (see code below).
  *
  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
  */
 
 @TeleOp(name="Basic: Autonomous Mode", group="Linear OpMode")
 public class BasicAutonomousMode extends LinearOpMode {
 
     // Declare OpMode members for each of the 4 motors.
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotor leftFrontDrive = null;
     private DcMotor leftBackDrive = null;
     private DcMotor rightFrontDrive = null;
     private DcMotor rightBackDrive = null;

     static final double     FORWARD_SPEED = 0.6;
     static final double     BACKWARD_SPEED = 0.6;
     static final double     STRAFE_SPEED = 0.6;
     static final double     TURN_SPEED    = 0.5;

     @Override
     public void runOpMode() {
 
         // Initialize the hardware variables. Note that the strings used here must correspond
         // to the names assigned during the robot configuration step on the DS or RC devices.
         leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
         leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
         rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
         rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
 
         leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
         leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
         rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
         rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

         telemetry.addData("Status", "Initialized");
         telemetry.update();
 
         waitForStart();
       
         // Test forward movement
         leftFrontDrive.setPower(FORWARD_SPEED);
         leftBackDrive.setPower(FORWARD_SPEED);
         rightFrontDrive.setPower(FORWARD_SPEED);
         rightBackDrive.setPower(FORWARD_SPEED);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
         }

         // Test backwards movement
         leftFrontDrive.setPower(BACKWARD_SPEED);
         leftBackDrive.setPower(BACKWARD_SPEED);
         rightFrontDrive.setPower(BACKWARD_SPEED);
         rightBackDrive.setPower(BACKWARD_SPEED);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
         }

         // Test turning right then left
         leftFrontDrive.setPower(FORWARD_SPEED);
         leftBackDrive.setPower(FORWARD_SPEED);
         rightFrontDrive.setPower(BACKWARD_SPEED);
         rightBackDrive.setPower(BACKWARD_SPEED);
         runtime.reset();
        
         leftFrontDrive.setPower(BACKWARD_SPEED);
         leftBackDrive.setPower(BACKWARD_SPEED);
         rightFrontDrive.setPower(FORWARD_SPEED);
         rightBackDrive.setPower(FORWARD_SPEED);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
         }
         
         // Test Strafing right then left
         leftFrontDrive.setPower(FORWARD_SPEED);
         leftBackDrive.setPower(BACKWARD_SPEED);
         rightFrontDrive.setPower(BACKWARD_SPEED);
         rightBackDrive.setPower(FORWARD_SPEED);
         runtime.reset();
        
         leftFrontDrive.setPower(BACKWARD_SPEED);
         leftBackDrive.setPower(FORWARD_SPEED);
         rightFrontDrive.setPower(FORWARD_SPEED);
         rightBackDrive.setPower(BACKWARD_SPEED);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Leg 4: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
         }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
         }
    }
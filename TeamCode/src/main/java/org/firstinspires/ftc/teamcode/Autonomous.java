package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.CustomHolonomicDrive.*;

/**
 * Plans for autonomous:
 * place specimen on bar
 * grab sample
 * sample in bucket
 * grab sample
 * sample in pucket
 * park
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Test")
public class Autonomous extends CustomHolonomicDrive {
    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private IMU imu = null;

    private Limelight3A limelight;

    private Follower follower;

    private Pose startPose = new Pose(10.000, 58.000, Point.CARTESIAN);

    private ArrayList<PathChain> paths = new ArrayList<>();
    private int pathIndex = 0;

    @Override
    public void init() {
        super.init();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        /*
         * Pipeline 0 is for the red sample
         * 1 is for the blue sample
         * 2 is for the yellow sample
         * 3 is for april tag 11
         * 4 is for april tag 12
         * 5 is for april tag 13
         * 6 is for april tag 14
         * 7 is for april tag 15
         * 8 is for april tag 16
         * 9 for all at once
         */

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start the limelight polling for data (getLatestResult() will return null without this)
        limelight.start();
        limelight.pipelineSwitch(9);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void start() {

        LLStatus limelightStatus = limelight.getStatus();
        telemetry.addData("Pipeline", "Index: %d, Type: %s", limelightStatus.getPipelineIndex(), limelightStatus.getPipelineType());

        LLResult limelightResult = limelight.getLatestResult();

        if (limelightResult != null) {
            limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
            // Access general information
            Pose3D botpose = limelightResult.getBotpose();

            double captureLatency = limelightResult.getCaptureLatency();
            double targetingLatency = limelightResult.getTargetingLatency();
            double parseLatency = limelightResult.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(limelightResult.getPythonOutput()));

            if (limelightResult.isValid()) {
                double targetX = limelightResult.getTx(); // How far left or right the target is (degrees)
                double targetY = limelightResult.getTy(); // How far up or down the target is (degrees)
                double targetArea = limelightResult.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", targetX);
                telemetry.addData("Target Y", targetY);
                telemetry.addData("Target Area", targetArea);

                if (botpose != null) {
                    double robotX = botpose.getPosition().x;
                    double robotY = botpose.getPosition().y;
                    double robotRotation = botpose.getOrientation().getYaw();
                    telemetry.addData("MT1 Location", "(" + robotX + ", " + robotY + ", " + robotRotation + ")");
                }

                telemetry.addData("Botpose", botpose.toString());

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = limelightResult.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = limelightResult.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Limelight", "No targets");
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
        telemetry.update();

        paths.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(startPose),
                                        new Point(35.000, 70.000, Point.CARTESIAN) // Drive to chamber
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(35.000, 70.000, Point.CARTESIAN),
                                        new Point(13.000, 19.000, Point.CARTESIAN) // Drive to closest sample
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build());

        paths.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(13.000, 19.000, Point.CARTESIAN),
                                        new Point(19.995, 124.173, Point.CARTESIAN) // Drive to bucket
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build());

        // Place sample in bucket

        paths.add(follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(
                                        new Point(19.995, 124.173, Point.CARTESIAN),
                                        new Point(10.922, 7.561, Point.CARTESIAN) // Park
                                )
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build());
    }

    @Override
    public void loop() {
        telemetry.addLine("In Loop");
        telemetry.addData("PathIndex: ", pathIndex);
        follower.update();
        if (follower.atParametricEnd()) {
            pathIndex++;
            if (pathIndex < paths.size()) {
                follower.followPath(paths.get(pathIndex));
            }
        }
        telemetry.update();
    }
}


/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: front_left_drive and front_right_drive, back_left_drive and back_right_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID.
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the // @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@TeleOp(name="Mecanum Drive To AprilTagFTC 1", group = "Concept")
// @Disabled
public class AprilTagtrackingFTC1 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    double DISTANCE = 36;
    final double DESIRED_DISTANCE = (DISTANCE); //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.012 ;   //  Turn Control "Gain". Increased by 20% (0.01 * 1.20)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.36;  //  Clip the turn speed to this max value (increased by 20% from 0.3)
    final double MAX_SHOOTER_POWER = 0.535; //  Max power for the shooter motor (53%)

    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right right drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel
    private DcMotorEx shooterMotor = null; //  Used to control shooter motor

    private DcMotor intakeMotor = null;
    private DcMotor transferMotor = null;//  Used to control transfer motor
    private CRServo frontrightservo = null;       //  Intake servo

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagSingleDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    private GoBildaPinpointDriver pinpoint = null;   // GoBilda Pinpoint Odometry Computer

    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize hardware mapping for all drive motors, shooter motor, transfer motor, and intake servo
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backleftmotor");
        backRightDrive = hardwareMap.get(DcMotor.class,"backrightmotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        // frontrightservo = hardwareMap.get(CRServo.class, "frontrightservo");

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor.setDirection(DcMotor.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize Pinpoint Odometry Computer and reset pose values to (0, 0, 0°) on INIT
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            pinpoint.update();
            telemetry.addData("Pinpoint Init", "Reset to X: 0.00 in, Y: 0.00 in");
        } catch (Exception e) {
            pinpoint = null;
            telemetry.addData("Pinpoint Status", "Error during init: " + e.getMessage());
        }

        // Loop during INIT phase: display odometry values on telemetry
        while (!isStarted() && !isStopRequested()) {
            displayOdometryTelemetry();
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream " + DESIRED_DISTANCE);
            telemetry.addData(">", "Touch START to start OpMode");
            telemetry.update();
            sleep(20);
        }

        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            if (aprilTag != null) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                        // Look to see if we have size info on this tag.
                        if (singleDetection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID < 0) || (singleDetection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = singleDetection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", singleDetection.id);
                            }
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", singleDetection.id);
                        }
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addLine(String.format("Found ID %d (%s)", desiredTag.id, desiredTag.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        desiredTag.ftcPose.x,
                        desiredTag.ftcPose.y,
                        desiredTag.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        desiredTag.ftcPose.pitch,

                        desiredTag.ftcPose.roll,
                        desiredTag.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        desiredTag.ftcPose.range,
                        desiredTag.ftcPose.bearing,
                        desiredTag.ftcPose.elevation));
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // Drive using Left Stick Y for forward/back drive
                drive  = -gamepad1.left_stick_y;          // Drive forward/backward based on Left Stick Y
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 2.0;  // Turn rate increased by 20% (40% max power, /2.5 instead of /3.0).

                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            // Mechanism Controls:
            // Right Bumper = Shooter & Transfer (0.51) - Transfer starts when shooter speed > 3000 RPM
            // Right Trigger = Disabled / Commented out
            // Left Trigger = Intake motor ONLY (.75)
            if (gamepad1.right_bumper) {
                shooterMotor.setPower(MAX_SHOOTER_POWER);
                double shooterRPM = Math.abs(shooterMotor.getVelocity(AngleUnit.DEGREES) / 6.0);
                if (shooterRPM > 27) {
                    transferMotor.setPower(.51  );
                    intakeMotor.setPower(.40);
                } else {
                    transferMotor.setPower(0);
                    intakeMotor.setPower(0);
                }
                telemetry.addData("Shooter RPM", "%.0f", shooterRPM);
                telemetry.addData("Shooter Mode (RB)", "Shooter: %.2f, Transfer: %s, Intake: %s",
                        MAX_SHOOTER_POWER,
                        (shooterRPM > 27 ? "0.51" : "Waiting"),
                        (shooterRPM > 27 ? "1.00" : "0.00"));
            }
            // Right Trigger code commented out:
            // else if (gamepad1.right_trigger > 0) {
            //     shooterMotor.setPower(0);
            //     transferMotor.setPower(1.0);
            //     intakeMotor.setPower(0);
            //     telemetry.addData("Transfer Mode (RT)", "1.00");
            // }
            else if (gamepad1.left_trigger > 0) {
                shooterMotor.setPower(0);
                transferMotor.setPower(0);
                intakeMotor.setPower(.50);
                telemetry.addData("Intake Mode (LT)", ".75");
            } else {
                shooterMotor.setPower(0);
                transferMotor.setPower(0);
                intakeMotor.setPower(0);
            }

            displayOdometryTelemetry();
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Helper method to display odometry / encoder telemetry
     */
    private void displayOdometryTelemetry() {
        if (pinpoint != null) {
            try {
                pinpoint.update();
                Pose2D pose = pinpoint.getPosition();
                telemetry.addData("X Pose", "%.2f in", pose.getX(DistanceUnit.INCH));
                telemetry.addData("Y Pose", "%.2f in", pose.getY(DistanceUnit.INCH));
                telemetry.addData("Heading Pose", "%.2f°", pose.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Pinpoint Pod X Ticks", pinpoint.getEncoderX());
                telemetry.addData("Pinpoint Pod Y Ticks", pinpoint.getEncoderY());
            } catch (Exception e) {
                telemetry.addData("Pinpoint Error", e.getMessage());
            }
        } else {
            telemetry.addData("Pinpoint Status", "NOT FOUND - check robot config for name 'pinpoint'");
        }

        if (frontLeftDrive != null && frontRightDrive != null && backLeftDrive != null && backRightDrive != null) {
            telemetry.addData("Motor Encoders", "FL: %d | FR: %d | BL: %d | BR: %d",
                    frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition(),
                    backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag library by using a builder.
        AprilTagLibrary.Builder tagLibraryBuilder = new AprilTagLibrary.Builder();

        // Add the official FTC game tags to the builder
        tagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Override or add tags 0-19 with the custom 6.75 inch size
        for (int id = 0; id < 20; id++) {
            tagLibraryBuilder.addTag(
                    id,
                    "Tag " + id,
                    6.75,
                    DistanceUnit.INCH);
        }
        AprilTagLibrary tagLibrary = tagLibraryBuilder.build();

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(tagLibrary)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new android.util.Size(640, 480))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoRed", group = "Autonomous")
public class AutoRed extends LinearOpMode {

    private Follower follower;

    // Mechanism hardware motors
    private DcMotorEx shooterMotor = null;
    private DcMotor intakeMotor = null;
    private DcMotor transferMotor = null;

    final double MAX_SHOOTER_POWER = 0.535; // Max power for the shooter motor (53%)

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8.5053, -90);
    private final Pose path1 = poseFactory.of(56, 23.3661, -90);
    private final Pose point2Start = poseFactory.of(56, 23.3661, 0);
    private final Pose point2 = poseFactory.of(36.2214, 23.3907, 0);
    private final Pose point3Start = poseFactory.of(36.2214, 23.3907, -60);
    private final Pose point3 = poseFactory.of(13.3517, 100.2441, -180);

    // Shooting command
    public Command shootCommand() {
        return sequential(
                // Step 1: Turn on shooter motor to spin up
                instant(() -> {
                    shooterMotor.setPower(MAX_SHOOTER_POWER);
                    transferMotor.setPower(0);
                    intakeMotor.setPower(0.2);
                }),
                // Step 2: Wait until shooter motor reaches target RPM (> 28)
                waitUntil(() -> {
                    double shooterRPM = Math.abs(shooterMotor.getVelocity(AngleUnit.DEGREES) / 6.0);
                    return shooterRPM > 28;
                }),
                // Step 3: Turn on transfer and intake to feed elements into shooter
                instant(() -> {
                    transferMotor.setPower(0.51);
                    intakeMotor.setPower(1.0);
                }),
                // Step 4: Run transfer/intake for 4000 milliseconds (4 seconds) to complete shooting
                waitMs(4000),
                // Step 5: Turn off all mechanism motors
                instant(() -> {
                    shooterMotor.setPower(0);
                    transferMotor.setPower(0);
                    intakeMotor.setPower(0);
                })
        );
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                shootCommand(),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Hardware initialization for drivetrain follower
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Hardware initialization for mechanism motors
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor.setDirection(DcMotor.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }
}

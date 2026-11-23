package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoRedWait", group = "Autonomous")
public class AutoRedWait extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses (Y = Forward/Backward, X = Left/Right, Heading 90° = Facing Forward)
    private final Pose start = poseFactory.of(57.481, 8.5268, 90);
    private final Pose point1 = poseFactory.of(57.4423, 25.226, 90);
    private final Pose point2Start = poseFactory.of(57.4423, 25.226, 90);
    private final Pose point2 = poseFactory.of(57.664, 9.6731, 90);
    private final Pose point3 = poseFactory.of(5.7405, 100.1823, 123);

    // Pause after each movement (milliseconds)
    private static final double WAIT_MS = 3000;

    // Turn-in-place settings
    private static final double TURN_TOLERANCE_DEG = 2.0; // close enough to the target heading
    private static final double TURN_TIMEOUT_MS = 2500;   // give up turning after this long

    // Autonomous routine:
    // forward to point1, wait, back to point2, wait,
    // turn in place to face point3, drive straight to point3, wait
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                waitMs(WAIT_MS),
                follow(follower, path2()),
                waitMs(WAIT_MS),
                turnTo(point2.withHeading(path3Heading())),
                follow(follower, path3()),
                waitMs(WAIT_MS)
        );
    }

    /**
     * Turns the robot in place to the given pose's heading (holding its X/Y),
     * finishing when the heading is within TURN_TOLERANCE_DEG or after TURN_TIMEOUT_MS.
     */
    private Command turnTo(Pose target) {
        return sequential(
                hold(follower, target),
                race(
                        waitUntil(() -> headingErrorDeg(target.heading()) < TURN_TOLERANCE_DEG),
                        waitMs(TURN_TIMEOUT_MS)
                )
        );
    }

    /** Absolute heading error in degrees, using the shortest way around the circle. */
    private double headingErrorDeg(double targetRad) {
        double diff = follower.pose().heading() - targetRad;
        diff = Math.atan2(Math.sin(diff), Math.cos(diff)); // wrap to [-pi, pi]
        return Math.abs(Math.toDegrees(diff));
    }

    /** Heading (radians) that points straight from point2 to point3. */
    private double path3Heading() {
        return Math.atan2(point3.y() - point2.y(), point3.x() - point2.x());
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        // Keep starting pose locked to starting coordinates during INIT
        while (!isStarted() && !isStopRequested()) {
            follower.setPose(start);
            follower.update();
            telemetry.addData("Status", "INIT - Locked to Starting Coordinates");
            telemetry.addData("X (in)", "%.2f", follower.pose().x());
            telemetry.addData("Y (in)", "%.2f", follower.pose().y());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        follower.setPose(start);
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.pose().heading()));

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.line(start, point1).linear(start.heading(), point1.heading());
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start.heading(), point2.heading());
    }

    // Drive straight to point3 facing the direction of travel (robot already turned)
    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }
}

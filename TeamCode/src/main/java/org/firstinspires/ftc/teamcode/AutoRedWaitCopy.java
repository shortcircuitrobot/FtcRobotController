package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.ConstantsBackup;

@Disabled
@Autonomous(name = "AutoRedWaitCopy", group = "Autonomous")
public class AutoRedWaitCopy extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses (Y = Forward/Backward, X = Left/Right, Heading 90° = Facing Forward)
    private final Pose start = poseFactory.of(57.481, 8.5268, 90);
    private final Pose point1 = poseFactory.of(57.4423, 25.226, 90);
    private final Pose point2Start = poseFactory.of(57.4423, 25.226, 90);
    private final Pose point2 = poseFactory.of(57.664, 9.6731, 90);
    private final Pose point3 = poseFactory.of(5.7405, 100.1823, -60.1579);

    // Autonomous routine: Forward to point1, backward to point2
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
                // , follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = ConstantsBackup.create(hardwareMap);
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

    public Path path3() {
        return Paths.line(point2, point3).linear(point2.heading(), point3.heading());
    }
}

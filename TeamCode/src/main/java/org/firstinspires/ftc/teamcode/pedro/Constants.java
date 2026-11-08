package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(hardwareMap, FConstants.class, LConstants.class);
    }
}

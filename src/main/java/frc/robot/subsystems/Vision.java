package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight";

    public boolean hasTarget() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);
    }
}
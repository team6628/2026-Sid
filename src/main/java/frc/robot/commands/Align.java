package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Align extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric driveRequest;

    private static final double kP = 0.04;
    private static final double TOLERANCE = 1.5; // degrees tolerance for alignment

    public Align(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
        m_drivetrain = drivetrain;
        driveRequest = drive;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tx = table.getEntry("tx").getDouble(0.0);  // horizontal offset
        double tv = table.getEntry("tv").getDouble(0.0);  // target valid (1 if any target detected)

        // Compute rotation inside a final variable to satisfy lambda
        final double rotationOutput = (tv > 0 && Math.abs(tx) > TOLERANCE) ? -tx * kP : 0.0;

        // Debug info
        System.out.println("[Align] tv: " + tv + ", tx: " + tx + ", rotationOutput: " + rotationOutput);

        // Apply rotation request with zero X/Y velocity
        m_drivetrain.applyRequest(() ->
                driveRequest.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(rotationOutput)
        );
    }

    @Override
    public boolean isFinished() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tx = table.getEntry("tx").getDouble(0.0);
        double tv = table.getEntry("tv").getDouble(0.0);

        // Finish when target exists and horizontal error is within tolerance
        return tv > 0 && Math.abs(tx) <= TOLERANCE;
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Align extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric driveRequest;

    private final double kP = 0.04; 
    private final double TOLERANCE = 1.5; // degrees tolerance for alignment

    public Align(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive) {
        m_drivetrain = drivetrain;
        driveRequest = drive;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tid = table.getEntry("tid").getDouble(0.0); // tag ID (Limelight 3)
        double tx  = table.getEntry("tl").getDouble(0.0);  // horizontal offset

        boolean hasTarget = tid > 0;

        // Debug print
        System.out.println("[Align] tid: " + tid + ", tx: " + tx);

        if (hasTarget && Math.abs(tx) > TOLERANCE) {
            double rotationOutput = -tx * kP;

            m_drivetrain.applyRequest(() ->
                driveRequest.withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(rotationOutput)
            );
        } else {
            m_drivetrain.applyRequest(() ->
                driveRequest.withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0)
            );
        }
    }

    @Override
    public boolean isFinished() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tid = table.getEntry("tid").getDouble(0.0);
        double tx  = table.getEntry("tl").getDouble(0.0);

        return tid > 0 && Math.abs(tx) <= TOLERANCE;
    }
}
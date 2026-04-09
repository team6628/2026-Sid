package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class Align extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.RobotCentric driveRequest;
    

    private static final double kP = 0.04;
    private static final double TOLERANCE = 1.5; // degrees tolerance for alignment
    private final Shooter shooter;

    public Align(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive, Shooter shooterSubsystem) {
    m_drivetrain = drivetrain;
    driveRequest = drive;
    shooter = shooterSubsystem;
    addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = table.getEntry("tv").getDouble(0.0);

        double tx = shooter.getCorrectedTX();

        final double rotationOutput =
            (tv > 0 && Math.abs(tx) > TOLERANCE) ? -tx * kP : 0.0;

        System.out.println("[Align] tv: " + tv + ", corrected tx: " + tx + ", rotationOutput: " + rotationOutput);

        m_drivetrain.setControl(
            driveRequest.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationOutput)
        );
    }

    @Override
    public boolean isFinished() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        double tv = table.getEntry("tv").getDouble(0.0);

        double tx = shooter.getCorrectedTX();

    return tv > 0 && Math.abs(tx) <= TOLERANCE;
    }
}
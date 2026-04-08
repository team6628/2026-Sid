package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Shake extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Timer xTimer = new Timer();
    private final Timer yTimer = new Timer();
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    private boolean xDirection = true;
    private boolean yDirection = true;

    // Adjustable parameters for shake strength
    private static final double TRANS_X_SPEED = 0.5;  // m/s, forward/backward
    private static final double TRANS_Y_SPEED = 0.5;  // m/s, left/right
    private static final double SWITCH_TIME_X = 0.12; // seconds between X flips
    private static final double SWITCH_TIME_Y = 0.15; // seconds between Y flips

    public Shake(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xDirection = true;
        yDirection = true;
        xTimer.restart();
        yTimer.restart();
    }

    @Override
    public void execute() {
        // Flip signs based on direction for independent axes
        if (xTimer.hasElapsed(SWITCH_TIME_X)) {
            xDirection = !xDirection;
            xTimer.restart();
        }

        if (yTimer.hasElapsed(SWITCH_TIME_Y)) {
            yDirection = !yDirection;
            yTimer.restart();
        }

        double xVel = xDirection ? TRANS_X_SPEED : -TRANS_X_SPEED;
        double yVel = yDirection ? TRANS_Y_SPEED : -TRANS_Y_SPEED;

        drivetrain.setControl(
                drive.withVelocityX(xVel)
                        .withVelocityY(yVel)
                        .withRotationalRate(0) // no rotation
        );
    }

    @Override
    public boolean isFinished() {
        return false; // continues while button held
    }

    @Override
    public void end(boolean interrupted) {
        // stop all motion when command ends
        drivetrain.setControl(
                drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)
        );
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Shake extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private int cycles = 0;
    private boolean direction = true;
    private Timer timer = new Timer();
    private final SwerveRequest.RobotCentric drive =
        new SwerveRequest.RobotCentric();

    public Shake(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        cycles = 0;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        double rot = direction ? 3.0 : -3.0;

        drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rot)
        );

        if (timer.hasElapsed(0.2)) {
            direction = !direction;
            cycles++;
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return cycles > 10;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
    drive.withVelocityX(0)
         .withVelocityY(0)
         .withRotationalRate(0)
);
    }
}
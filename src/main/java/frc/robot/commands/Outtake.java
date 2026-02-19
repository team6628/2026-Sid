package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Outtake extends Command {

    private final Shooter shooter;

    public Outtake(Shooter subsystem) {
        shooter = subsystem;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.outtake();
    }

    @Override
    public void execute() {
        // Nothing required here
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while held
    }
}

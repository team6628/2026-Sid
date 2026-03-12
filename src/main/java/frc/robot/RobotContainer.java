package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.Dump;

public class RobotContainer {

    /* ===================== DRIVETRAIN ===================== */

    private final double MaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.RobotCentric drive =
            new SwerveRequest.RobotCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandJoystick joystick = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain =
            TunerConstants.createDrivetrain();

    /* ===================== SUBSYSTEMS ===================== */

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake(shooter);
    private final Outtake outtake = new Outtake(shooter);
    private final Dump dump = new Dump(shooter);

    /* ===================== AUTO ===================== */

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        configureAutos();
    }

    /* ======================================================= */
    /* ===================== BUTTON BINDINGS ================= */
    /* ======================================================= */

    private void configureBindings() {

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-joystick.getY() * MaxSpeed)
                             .withVelocityY(-joystick.getX() * MaxSpeed)
                             .withRotationalRate(-joystick.getZ() * MaxAngularRate)
                )
        );

        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> new SwerveRequest.Idle())
                          .ignoringDisable(true)
        );

        joystick.button(7).and(joystick.button(4))
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        joystick.button(7).and(joystick.button(3))
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        joystick.button(8).and(joystick.button(4))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        joystick.button(8).and(joystick.button(3))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.button(1).whileTrue(intake);
        joystick.button(2).whileTrue(outtake);
        joystick.button(3).whileTrue(dump);
    }

    /* ======================================================= */
    /* ===================== AUTONOMOUS ====================== */
    /* ======================================================= */

    private void configureAutos() {

        /* ---- Register Named Commands ---- */
        NamedCommands.registerCommand("Intake", new Intake(shooter).withTimeout(4.0));
        NamedCommands.registerCommand("Outtake", new Outtake(shooter).withTimeout(4.0));

        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    () -> drivetrain.getState().Pose,
                    drivetrain::resetPose,

                    () -> drivetrain.getKinematics()
                            .toChassisSpeeds(drivetrain.getState().ModuleStates),

                    (speeds, feedforwards) ->
                            drivetrain.setControl(
                                    drive.withVelocityX(speeds.vxMetersPerSecond)
                                         .withVelocityY(speeds.vyMetersPerSecond)
                                         .withRotationalRate(speeds.omegaRadiansPerSecond)
                            ),

                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)
                    ),

                    config,

                    () -> DriverStation.getAlliance()
                            .map(a -> a == DriverStation.Alliance.Red)
                            .orElse(false),

                    drivetrain
            );

        } catch (Exception e) {
            DriverStation.reportError("Failed to configure AutoBuilder", e.getStackTrace());
        }

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
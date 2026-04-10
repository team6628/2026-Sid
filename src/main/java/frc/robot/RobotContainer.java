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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.Dump;
import frc.robot.commands.Shake;
import frc.robot.commands.Align;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class RobotContainer {

    /* ===================== DRIVETRAIN ===================== */

    private final double MaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate =
            RotationsPerSecond.of(2.0).in(RadiansPerSecond);

    private final SwerveRequest.RobotCentric drive =
            new SwerveRequest.RobotCentric()
                    .withDeadband(MaxSpeed * 0.08)
                    .withRotationalDeadband(MaxAngularRate * 0.08)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandJoystick joystick = new CommandJoystick(0);

    public final CommandSwerveDrivetrain drivetrain =
            TunerConstants.createDrivetrain();

    /* ===================== SUBSYSTEMS ===================== */

    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake(shooter);
    private final Outtake outtake = new Outtake(shooter, drivetrain);
    private final Dump dump = new Dump(shooter);
    private final Shake shake = new Shake(drivetrain);
    private final Align align = new Align(drivetrain, shooter);
    private boolean override = false;

    /* ===================== AUTO ===================== */

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();
        configureAutos();

        /*================== CAMERA =======================*/
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
        camera.setFPS(20);

        /*================== PRINT LOG ====================*/
        new Thread(() -> {
            while (true) {
                SmartDashboard.putNumber("Battery Voltage",
                        edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    /* ======================================================= */
    /* ===================== BUTTON BINDINGS ================= */
    /* ======================================================= */

    private void configureBindings() {

        // Default command
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-joystick.getY() * MaxSpeed)
                             .withVelocityY(-joystick.getX() * MaxSpeed)
                             .withRotationalRate(-joystick.getZ() * MaxAngularRate)
                )
        );

        // RobotModeTriggers.disabled().whileTrue(...)

        /* ================= SYSID ================= */

        joystick.button(7).and(joystick.button(4))
                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

        joystick.button(7).and(joystick.button(3))
                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        joystick.button(8).and(joystick.button(4))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        joystick.button(8).and(joystick.button(3))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        /* ================= MECHANISMS ================= */

        joystick.button(1).whileTrue(intake);
        joystick.button(4).whileTrue(outtake);
        joystick.button(3).whileTrue(dump);
        joystick.button(10).whileTrue(shake);
        joystick.button(2).onTrue(
        Commands.runOnce(() -> override = !override));
        joystick.button(4)
        .and(() -> override)
        .whileTrue(align);
    }

    /* ======================================================= */
    /* ===================== AUTONOMOUS ====================== */
    /* ======================================================= */

    private void configureAutos() {

        NamedCommands.registerCommand("Intake", new Intake(shooter).withTimeout(4.0));
        NamedCommands.registerCommand("Outtake", new Outtake(shooter, drivetrain).withTimeout(4.0));
        NamedCommands.registerCommand("Shake", new Shake(drivetrain).withTimeout(4.0));
        NamedCommands.registerCommand("Align", new Align(drivetrain, shooter).withTimeout(4.0));

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
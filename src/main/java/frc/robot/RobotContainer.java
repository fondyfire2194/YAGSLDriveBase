// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.auto.DriveToNearestBlueReefZone;
import frc.robot.commands.swervedrive.auto.DriveToNearestCoralStation;
import frc.robot.commands.swervedrive.auto.DriveToNearestRedReefZone;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import monologue.Logged;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {

  SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  Trigger reefZoneChange = new Trigger(() -> drivebase.reefZone != drivebase.reefZoneLast);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in
  // configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
          OperatorConstants.DEADBAND),
      () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
          OperatorConstants.RIGHT_X_DEADBAND),
      driverXbox.getHID()::getYButtonPressed,
      driverXbox.getHID()::getAButtonPressed,
      driverXbox.getHID()::getXButtonPressed,
      driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) * Math.PI)
          * (Math.PI * 2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) * Math.PI)
              *
              (Math.PI * 2))
      .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  public final LimelightVision m_llv = new LimelightVision();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    reefZoneChange.onTrue(rumble(driverXbox, RumbleType.kLeftRumble, 1))
        .onTrue(new InstantCommand(() -> drivebase.reefZoneLast = drivebase.reefZone));

    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("One Path", drivebase.getAutonomousCommand("One Path"));
    autoChooser.addOption("Two Path", drivebase.getAutonomousCommand("Two Path"));
    autoChooser.addOption("Three Path", drivebase.getAutonomousCommand("Three Path"));
   
    SmartDashboard.putData("PPAutoChooser", autoChooser);

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // (Condition) ? Return-On-True : Return-on-False
    // drivebase.setDefaultCommand(
    // !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle :
    // driveFieldOrientedDirectAngleSim);
    drivebase.setDefaultCommand(drivebase.driveCommand(
        () -> -driverXbox.getLeftY(),
        () -> -driverXbox.getLeftX(),
        () -> -driverXbox.getRightX()));

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(8, 4, new Rotation2d()))));
    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(Commands.none());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().whileTrue(drivebase.sysIdAngleMotorCommand());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(6, 3.8), Rotation2d.fromDegrees(180))));
      driverXbox.y().whileTrue(Commands.none());
      driverXbox.start().onTrue(drivebase.centerModulesCommand());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(
          Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(7.372, 6.692, new Rotation2d(Math.PI)))));

      driverXbox.povLeft().onTrue(
          new ConditionalCommand(
              new DriveToNearestRedReefZone(drivebase, true),
              new DriveToNearestBlueReefZone(drivebase, true),
              () -> isRedAlliance()));

      driverXbox.povRight().onTrue(
          new ConditionalCommand(
              new DriveToNearestRedReefZone(drivebase, false),
              new DriveToNearestBlueReefZone(drivebase, false),
              () -> isRedAlliance()));

      driverXbox.povUp().onTrue(
          new DriveToNearestCoralStation(drivebase));

          driverXbox.povDown().onTrue(Commands.none());
        

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(1, 4, new Rotation2d())));
    return autoChooser.getSelected();
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  private boolean isRedAlliance() {

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public Command rumble(CommandXboxController controller, RumbleType type, double timeout) {
    return Commands.sequence(
        Commands.race(
            Commands.either(
                Commands.run(() -> controller.getHID().setRumble(type,
                    1.0)),
                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                    "BUZZ")),
                () -> RobotBase.isReal()),
            Commands.waitSeconds(timeout)),
        Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

  }
}

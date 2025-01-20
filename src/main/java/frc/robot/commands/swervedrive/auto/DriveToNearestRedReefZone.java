// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestRedReefZone extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  Pose2d robotPose;
  double robotX;
  double robotY;
  double robotHeading;
  double yLimitForXHGZone;

  boolean exit;
  int tst;

  boolean m_leftSide;
  double plusYBorder;
  double minusYBorder;

  public DriveToNearestRedReefZone(SwerveSubsystem swerve, boolean leftSide) {
    m_swerve = swerve;
    m_leftSide = leftSide;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    exit = false;

    SmartDashboard.putBoolean("REDRUNNING", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    exit = false;
    m_swerve.reefZone = 0;
    robotPose = m_swerve.getPose();
    robotX = robotPose.getX();
    robotY = robotPose.getY();
    robotHeading = robotPose.getRotation().getDegrees();

    if (checkGHZone()) {
      m_swerve.reefZone = 1;
      exit = true;
    }

    if (!exit && checkABZone()) {
      m_swerve.reefZone = 4;
      exit = true;
    }

    if (!exit && checkIJZone()) {
      m_swerve.reefZone = 2;
      exit = true;
    }

    if (!exit && checkKLZone()) {
      m_swerve.reefZone = 3;
      exit = true;
    }

    if (!exit && checkCDZone()) {
      m_swerve.reefZone = 5;
      exit = true;
    }

    if (!exit && checkEFZone()) {
      m_swerve.reefZone = 6;
      exit = true;
    }

    if (m_swerve.reefZone != 0) {

      m_swerve.reefZoneTag = FieldConstants.redReefTags[m_swerve.reefZone];

      int tagNumber = FieldConstants.redReefTags[m_swerve.reefZone];

      m_swerve.reefTargetPose = m_swerve.getTagPose(tagNumber).toPose2d();

      double baseOffset = RobotConstants.placementOffset + RobotConstants.ROBOT_LENGTH / 2;

      if (m_swerve.isRedAlliance())
        baseOffset = -baseOffset;

      Translation2d tl2d = new Translation2d(-baseOffset, FieldConstants.reefOffset);
      if (m_leftSide)
        tl2d = new Translation2d(baseOffset, -FieldConstants.reefOffset);
      Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));

      m_swerve.reefFinalTargetPose = m_swerve.reefTargetPose.transformBy(tr2d);

      m_swerve.plusBorderPose = new Pose2d(robotX, plusYBorder, new Rotation2d());
      m_swerve.minusBorderPose = new Pose2d(robotX, minusYBorder, new Rotation2d());

       m_swerve.driveToPose(m_swerve.reefFinalTargetPose).schedule();
    }

  }

  boolean checkGHZone() {
    plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth /FieldConstants.reefSideWidthDiv
        + (FieldConstants.redReefGHEdgeFromCenterFieldX - robotX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));
    minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth /FieldConstants.reefSideWidthDiv
        - (FieldConstants.redReefGHEdgeFromCenterFieldX - robotX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));

    boolean borderX = robotX > FieldConstants.FIELD_LENGTH / 2
        && robotX < FieldConstants.redReefGHEdgeFromCenterFieldX;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkABZone() {
    plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth /FieldConstants.reefSideWidthDiv
        + (robotX - FieldConstants.redReefABEdgeFromCenterFieldX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));
    minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth /FieldConstants.reefSideWidthDiv
        - (robotX - FieldConstants.redReefABEdgeFromCenterFieldX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));

    boolean borderX = robotX > FieldConstants.redReefABEdgeFromCenterFieldX;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkCDZone() {
    return robotX > FieldConstants.redReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkIJZone() {
    return robotX < FieldConstants.redReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkKLZone() {
    return robotX > FieldConstants.redReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkEFZone() {
    return robotX < FieldConstants.redReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exit;
  }
}

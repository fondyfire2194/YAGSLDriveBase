// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindCurrentReefZoneRed extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  Pose2d robotPose;
  double robotX;
  double robotY;
  double robotHeading;
  double yLimitForXHGZone;

  boolean zoneFound;
  int tst;
  

  double plusYBorder;
  double minusYBorder;

  public FindCurrentReefZoneRed(SwerveSubsystem swerve) {
    m_swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    zoneFound = false;

    SmartDashboard.putBoolean("REDRUNNING", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    zoneFound = false;
    m_swerve.reefZoneTag = 0;
    robotPose = m_swerve.getPose();
    robotX = robotPose.getX();
    robotY = robotPose.getY();
    robotHeading = robotPose.getRotation().getDegrees();

    if (checkGHZone()) {
      m_swerve.reefZoneTag = 10;
      zoneFound = true;
    }

    if (!zoneFound && checkABZone()) {
      m_swerve.reefZoneTag = 7;
      zoneFound = true;
    }
 if (!zoneFound && checkCDZone()) {
      m_swerve.reefZoneTag = 8;
      zoneFound = true;
    }

    if (!zoneFound && checkEFZone()) {
      m_swerve.reefZoneTag = 9;
      zoneFound = true;
    }
    if (!zoneFound && checkIJZone()) {
      m_swerve.reefZoneTag = 11;
      zoneFound = true;
    }

    if (!zoneFound && checkKLZone()) {
      m_swerve.reefZoneTag = 6;
      zoneFound = true;
    }

   

    int tagNumber = m_swerve.reefZoneTag;

    m_swerve.reefTargetPose = m_swerve.getTagPose(tagNumber).toPose2d();

    m_swerve.plusBorderPose = new Pose2d(robotX, plusYBorder, new Rotation2d());
    m_swerve.minusBorderPose = new Pose2d(robotX, minusYBorder, new Rotation2d());

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
    minusYBorder = FieldConstants.FIELD_WIDTH / 2 -FieldConstants.reefSideWidth /FieldConstants.reefSideWidthDiv
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

 boolean checkEFZone() {
    return robotX < FieldConstants.redReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkIJZone() {
    return robotX < FieldConstants.redReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkKLZone() {
    return robotX > FieldConstants.redReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

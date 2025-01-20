// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindCurrentReefZoneBlue extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  Pose2d robotPose;
  double robotX;
  double robotY;
  double robotHeading;
  double yLimitForXHGZone;

  boolean zoneFound;
  int tst;
  double yLimitAngle = 60;
  

  public FindCurrentReefZoneBlue(SwerveSubsystem swerve) {
    m_swerve = swerve;
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


    SmartDashboard.putBoolean("BLUERUNNING", true);
    zoneFound = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    zoneFound = false;
    m_swerve.reefZone = 0;
    robotPose = m_swerve.getPose();
    robotX = robotPose.getX();
    robotY = robotPose.getY();
    robotHeading = robotPose.getRotation().getDegrees();

    if (checkGHZone()) {
      m_swerve.reefZone = 1;
      zoneFound = true;
    }

    if (!zoneFound && checkABZone()) {
      m_swerve.reefZone = 4;
      zoneFound = true;
    }

    if (!zoneFound && checkIJZone()) {
      m_swerve.reefZone = 5;
      zoneFound = true;
    }

    if (!zoneFound && checkKLZone()) {
      m_swerve.reefZone = 6;
      zoneFound = true;
    }

    if (!zoneFound && checkCDZone()) {
      m_swerve.reefZone = 3;
      zoneFound = true;
    }

    if (!zoneFound && checkEFZone()) {
      m_swerve.reefZone = 2;
      zoneFound = true;
    }
  }

  boolean checkGHZone() {

    double plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / 2
        + (robotX - FieldConstants.blueReefGHEdgeFromFieldOrigin) *
            Math.tan(Units.degreesToRadians(yLimitAngle));
    double minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / 2
        - (robotX - FieldConstants.blueReefGHEdgeFromFieldOrigin) *
            Math.tan(Units.degreesToRadians(yLimitAngle));

    boolean borderX = robotX < FieldConstants.FIELD_LENGTH / 2
        && robotX > FieldConstants.blueReefGHEdgeFromFieldOrigin;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkABZone() {
    double plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / 2
        + (FieldConstants.blueReefABEdgeFromFieldOrigin - robotX) *
            Math.tan(Units.degreesToRadians(yLimitAngle));
    double minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / 2
        - (FieldConstants.blueReefABEdgeFromFieldOrigin - robotX) *
            Math.tan(Units.degreesToRadians(yLimitAngle));

    boolean borderX = robotX < FieldConstants.blueReefABEdgeFromFieldOrigin;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkCDZone() {
    return robotX < FieldConstants.blueReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkEFZone() {
    return robotX > FieldConstants.blueReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkIJZone() {
    return robotX < FieldConstants.blueReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkKLZone() {
    return robotX > FieldConstants.blueReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
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

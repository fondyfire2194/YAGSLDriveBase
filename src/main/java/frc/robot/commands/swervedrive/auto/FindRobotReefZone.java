// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindRobotReefZone extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  Pose2d robotPose;
  double robotX;
  double robotY;
  double robotHeading;
  double yLimitForXHGZone;
  boolean inABZone;
  boolean inCDZone;
  boolean inEFZone;
  boolean inGHZone;
  boolean inIJZone;
  boolean inKLZone;
  boolean redAlliance;
  boolean exit;
  int tst;

  public FindRobotReefZone(SwerveSubsystem swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    exit = false;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      redAlliance = alliance.get() == DriverStation.Alliance.Red;
    }

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

    if (redAlliance) {

      if (checkGHZone()) {
        m_swerve.reefZone = 1;
        exit = true;

      }

      if (!exit && checkABZone()) {
        m_swerve.reefZone = 4;
        exit = true;
      }

      if (!exit && checkCDZone()) {
        m_swerve.reefZone = 3;
        exit = true;
      }

      if (!exit && checkEFZone()) {
        m_swerve.reefZone = 2;
        exit = true;
      }

      if (!exit && checkIJZone()) {
        m_swerve.reefZone = 5;
        exit = true;
      }

      if (!exit && checkKLZone()) {
        m_swerve.reefZone = 6;
        exit = true;
      }
    }
  }

  boolean checkGHZone() {
    double plusYBorder = FieldConstants.FIELD_WIDTH / 2 
        + (FieldConstants.reefGHEdgeFromCenterFieldX + RobotConstants.ROBOT_LENGTH / 2 - robotX) *
            Math.sin(Units.degreesToRadians(60));
    double minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / 2
        - (FieldConstants.reefGHEdgeFromCenterFieldX - robotX) *
            Math.sin(Units.degreesToRadians(60));

    boolean borderX = robotX > FieldConstants.FIELD_LENGTH / 2 && robotX < FieldConstants.reefGHEdgeFromCenterFieldX;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkABZone() {
    double plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / 2
        + (robotX - FieldConstants.reefABEdgeFromCenterFieldX) *
            Math.sin(Units.degreesToRadians(60));
    double minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / 2
        - (robotX - FieldConstants.reefABEdgeFromCenterFieldX) *
            Math.sin(Units.degreesToRadians(60));

    boolean borderX = robotX > FieldConstants.reefABEdgeFromCenterFieldX;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkCDZone() {
    return robotX < FieldConstants.reefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkEFZone() {
    return robotX > FieldConstants.reefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkIJZone() {
    return robotX > FieldConstants.reefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkKLZone() {
    return robotX < FieldConstants.reefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_swerve.reefZone != 0)
      m_swerve.reefTargetPose = m_swerve.redReefPoses[m_swerve.reefZone].toPose2d();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

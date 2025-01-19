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
import frc.robot.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindBlueRobotReefZone extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  Pose2d robotPose;
  double robotX;
  double robotY;
  double robotHeading;
  double yLimitForXHGZone;

  boolean exit;
  int tst;
  double yLimitAngle = 60;
  boolean m_leftSide;

  public FindBlueRobotReefZone(SwerveSubsystem swerve, boolean leftSide) {
    m_swerve = swerve;
    m_leftSide = leftSide;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putBoolean("BLUERUNNING", true);
    SmartDashboard.putNumber("BORDERBLMID", FieldConstants.blueReefMidFromCenterFieldX);
    exit = false;
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
      m_swerve.reefZone = 5;
      exit = true;
    }

    if (!exit && checkKLZone()) {
      m_swerve.reefZone = 6;
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
    if (m_swerve.reefZone != 0) {

      m_swerve.reefZoneTag = FieldConstants.blueReefTags[m_swerve.reefZone];
      int tagNumber = FieldConstants.blueReefTags[m_swerve.reefZone];
      m_swerve.reefTargetPose = m_swerve.getTagPose(tagNumber).toPose2d();

      double baseOffset = RobotConstants.placementOffset + RobotConstants.ROBOT_LENGTH / 2;

      Translation2d tl2d = new Translation2d(-baseOffset, FieldConstants.reefOffset);
      if (m_leftSide)
        tl2d = new Translation2d(baseOffset, -FieldConstants.reefOffset);
      Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));

      m_swerve.reefTargetPose1 = m_swerve.reefTargetPose.transformBy(tr2d);

      m_swerve.driveToPose(m_swerve.reefTargetPose1).schedule();
    }
  }

  boolean checkGHZone() {

    double xdiff = robotX - FieldConstants.blueReefGHEdgeFromFieldOrigin;
    SmartDashboard.putNumber("BorderXDiff", xdiff);

    double plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / 2
        + (robotX - FieldConstants.blueReefGHEdgeFromFieldOrigin) *
            Math.tan(Units.degreesToRadians(yLimitAngle));
    double minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / 2
        - (robotX - FieldConstants.blueReefGHEdgeFromFieldOrigin) *
            Math.tan(Units.degreesToRadians(yLimitAngle));

    SmartDashboard.putNumber("BorderYMinus", minusYBorder);
    SmartDashboard.putNumber("BorderYPlus", plusYBorder);

    boolean borderX = robotX < FieldConstants.FIELD_LENGTH / 2
        && robotX > FieldConstants.blueReefGHEdgeFromFieldOrigin;
    SmartDashboard.putBoolean("BorderX", borderX);
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

    SmartDashboard.putNumber("BorderYMinus", minusYBorder);
    SmartDashboard.putNumber("BorderYPlus", plusYBorder);

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
    return exit;
  }
}

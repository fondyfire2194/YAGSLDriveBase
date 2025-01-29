// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectNoteIntakeToShooter extends Command {
  /** Creates a new DetectNoteIntakeToShooter. */
  private final ShooterSubsystem m_shooter;
  private final boolean m_closedLoop;
  private final boolean m_useRPM;
 

  private double maxRPM = 5700;

  public DetectNoteIntakeToShooter(ShooterSubsystem shooter, boolean closedLoop, boolean useRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_closedLoop = closedLoop;
    m_useRPM = useRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putNumber("ShooterRPM", 500);
    SmartDashboard.putNumber("ShooterAmpDetect", 5);
    SmartDashboard.putNumber("ShooterRPMDetect", 300);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double rpm = SmartDashboard.getNumber("ShooterRPM", 5000);


    if (!m_closedLoop) {
      double val = rpm / maxRPM;
      m_shooter.topRoller.setVoltage(-val * RobotController.getBatteryVoltage());
      m_shooter.bottomRoller.setVoltage(-val * RobotController.getBatteryVoltage());
    } else {
      m_shooter.setCommandRPM(RPM.of(-rpm));
      m_shooter.setRunShooter();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.resetRunShooter();
    m_shooter.topRoller.set(0);
    m_shooter.bottomRoller.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

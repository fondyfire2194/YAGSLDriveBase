// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Old;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TransferIntakeToSensor extends Command {
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private final SwerveSubsystem m_swerve;
  private final double m_noteMissedTime;
  private Timer endTimer = new Timer();

  /** Creates a new TransferIntakeToSensor. */
  public TransferIntakeToSensor(TransferSubsystem transfer, IntakeSubsystem intake, SwerveSubsystem swerve,
      double noteMissedTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transfer = transfer;
    m_swerve = swerve;
    m_intake = intake;
    m_noteMissedTime = noteMissedTime;
  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endTimer.reset();
    endTimer.start();
    m_intake.noteMissed = false;
    if (RobotBase.isReal())
      m_intake.resetIsIntakingSim();
    if (RobotBase.isSimulation()) {
      m_intake.isIntaking4 = m_intake.isIntaking3;
      m_intake.isIntaking3 = m_intake.isIntaking2;
      m_intake.isIntaking2 = m_intake.isIntaking1;
    }
    m_intake.isIntaking1 = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    SmartDashboard.putNumber("Transfer/NoteMissedTime", endTimer.get());
    m_transfer.runToSensor();
   

    if (RobotBase.isSimulation() 
        && (DriverStation.isAutonomousEnabled() || endTimer.hasElapsed(2))) {
      if (m_intake.isIntaking1 && !m_intake.isIntaking2 && !m_intake.isIntaking3 && !m_intake.isIntaking4
          && !m_transfer.skipFirstNoteInSim) {
        m_transfer.simnoteatintake = true;
      }
      if (m_intake.isIntaking2 && !m_intake.isIntaking3 && !m_intake.isIntaking4
          && !m_transfer.skipSecondNoteInSim) {
        m_transfer.simnoteatintake = true;
      }
      if (m_intake.isIntaking3 && !m_intake.isIntaking4
          && !m_transfer.skipThirdNoteInSim) {
        m_transfer.simnoteatintake = true;
      }
      if (m_intake.isIntaking4
          && !m_transfer.skipFourthNoteInSim) {
        m_transfer.simnoteatintake = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    if (DriverStation.isTeleopEnabled()) {
      m_intake.stopMotor();

    }
    m_transfer.enableLimitSwitch(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_transfer.noteAtIntake() || m_intake.noteMissed || RobotBase.isSimulation() && m_transfer.simnoteatintake;
  }
}

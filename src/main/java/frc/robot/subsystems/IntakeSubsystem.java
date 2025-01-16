// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public SparkMax intakeMotor;
  public SparkClosedLoopController intakeController;
  private SparkMaxConfig intakeConfig;

  private boolean runIntake;
  private boolean reverseIntake;
  public boolean jogging;

  private AngularVelocity commandrpm;

  public boolean noteMissed;
  public boolean intakeMotorConnected;

  public boolean isIntaking1;

  public boolean isIntaking2;

  public boolean isIntaking3;

  public boolean isIntaking4;

  public final double intakeKp = 2e-4;
  public  final double intakeKi = 0.0;
  public  final double intakeKd = 0.00;
  public  final double intakeKFF = .95 / 5700;;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(18, MotorType.kBrushless);
    intakeController = intakeMotor.getClosedLoopController();
    intakeConfig = new SparkMaxConfig();

    intakeConfig.inverted(false)
        .idleMode(IdleMode.kBrake);

    intakeConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    intakeConfig.closedLoop
        .velocityFF(intakeKFF)
        .pid(intakeKp, intakeKi, intakeKd);

    intakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

    intakeConfig.signals.primaryEncoderPositionPeriodMs(5);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void stopMotor() {
    intakeMotor.stopMotor();
    intakeController.setReference(0, ControlType.kVelocity);
    resetRunIntake();
    resetReverseIntake();
    commandrpm = RPM.of(0);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopMotor());
  }

  public Command startIntakeCommand() {
    return Commands.runOnce(() -> setRunIntake());
  }

  public Command reverseIntakeCommand() {
    return Commands.runOnce(() -> setReverseIntake());
  }

  public void setRunIntake() {
    runIntake = true;
    reverseIntake = false;
  }

  public void resetRunIntake() {
    runIntake = false;
  }

  public boolean getRunIntake() {
    return runIntake;
  }

  public void setReverseIntake() {
    reverseIntake = true;
    runIntake = false;
  }

  public void resetReverseIntake() {
    reverseIntake = false;
  }

  public boolean getReverseIntake() {
    return reverseIntake;
  }

  public void resetIsIntakingSim() {
    isIntaking1 = false;
    isIntaking2 = false;
    isIntaking3 = false;
    isIntaking4 = false;
  }

  public double getRPM() {
    return intakeMotor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (runIntake && !reverseIntake) {
      commandrpm = RPM.of(4500);// Pref.getPref("IntakeSpeed");
      runAtVelocity(commandrpm.in(RPM));
    }

    if (reverseIntake && !runIntake) {
      commandrpm = RPM.of(4500).times(-1);// Pref.getPref("IntakeSpeed");
      runAtVelocity(commandrpm.in(RPM));
    }

    if (!runIntake && !reverseIntake && !jogging) {
      stopMotor();
    }

    if (!intakeMotorConnected) {
      intakeMotorConnected = checkMotorCanOK(intakeMotor);
      SmartDashboard.putBoolean("Intake//OKIntakeMotor", intakeMotorConnected);
    }
  }

  private boolean checkMotorCanOK(SparkMax motor) {

    return RobotBase.isSimulation();
  }

  public Command testCan() {
    return runOnce(() -> intakeMotorConnected = false);
  }

  private void runAtVelocity(double rpm) {
    intakeController.setReference(rpm, ControlType.kVelocity);
  }

  public void reverseMotor() {
    runAtVelocity(-500);
  }

  public double getAmps() {
    return intakeMotor.getOutputCurrent();
  }

  public void setPID() {
    intakeConfig.closedLoop.p(intakeKp);
    intakeConfig.closedLoop.velocityFF(intakeKFF);

  }

  public boolean getStickyFaults() {
    return intakeMotor.hasActiveFault();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.runOnce(() -> intakeMotor.clearFaults());
  }

}

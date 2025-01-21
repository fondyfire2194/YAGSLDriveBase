// Copytop (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;



public class ShooterSubsystem extends SubsystemBase {

  public SparkMax topRoller;
  public SparkClosedLoopController topController;

  public SparkMax bottomRoller;

  public SparkClosedLoopController bottomController;

  private SparkMaxConfig topConfig;
  private SparkMaxConfig bottomConfig;

  public double topCommandRPM =500;
  public double bottomCommandRPM = 500;

  private double topSimRPM = 500;
  private double bottomSimRPM = 500;

  private boolean shootersatspeed;

  public double shootertolerancepct = 10;

  private boolean runShooterVel;

  private SlewRateLimiter topSpeedLimiter = new SlewRateLimiter(15000);
  private SlewRateLimiter bottomSpeedLimiter = new SlewRateLimiter(15000);
  public boolean topMotorConnected;
  public boolean bottomMotorConnected;

  public boolean okTriggerLobShot;

  public boolean okTriggerSpeakerShot;

    public static final double topShooterKP = 3e-4;
                public  final double topShooterKI = 0;
                public  final double topShooterKD = 0.02;
                public  final double topShooterKFF = 1.0 / 5700;
                public  final double bottomShooterKP = 3e-4;
                public  final double bottomShooterKI = 0;
                public  final double bottomShooterKD = .02;
                public  final double bottomShooterKFF = 1.0 / 5700;
                public  final double voltageComp = 12;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {

    topRoller = new SparkMax(17, MotorType.kBrushless);
    topController = topRoller.getClosedLoopController();

    topConfig = new SparkMaxConfig();
    bottomConfig = new SparkMaxConfig();

    bottomRoller = new SparkMax(16, MotorType.kBrushless);

    bottomController = bottomRoller.getClosedLoopController();

    topConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    topConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    topConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(topShooterKFF)
        .outputRange(0, 1)
        .pid(topShooterKP, topShooterKI, topShooterKD);
    topConfig.signals.primaryEncoderPositionPeriodMs(5);
    topRoller.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    bottomConfig
        .idleMode(IdleMode.kBrake)
        .inverted(true);
    bottomConfig.encoder
        // .setOutputRange()
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    bottomConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .velocityFF(topShooterKFF)
        .outputRange(0, 1)
        .pid(topShooterKP, topShooterKI, topShooterKD);
    bottomConfig.signals.primaryEncoderPositionPeriodMs(5);
    bottomRoller.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    okTriggerLobShot = true;
    okTriggerSpeakerShot = false;
  }

  public void stopMotors() {
    runShooterVel = false;
    // topController.setReference(0, ControlType.kVelocity);
    bottomController.setReference(0, ControlType.kVelocity);
    topRoller.stopMotor();
    bottomRoller.stopMotor();
    topSpeedLimiter.reset(0);
    bottomSpeedLimiter.reset(0);
    topSimRPM = 0;
    bottomSimRPM = 0;
  }

  public Command stopShooterCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> stopMotors()),
        Commands.runOnce(() -> topCommandRPM =0),
        Commands.runOnce(() -> bottomCommandRPM =0));

  }

  public void setRPMTolerancePCT(double pct) {
    if (pct < 10 || pct > 25)
      pct = 10;
    shootertolerancepct = pct;
  }

  public void setRPMTolerancePCT() {
    shootertolerancepct = 10;
  }

  public Command startShooterCommand(double rpm) {
    return Commands.run(() -> startShooter(rpm))
        .until(() -> bothAtSpeed());
  }

  public Command startShooterCommand(double toprpm, double bottomrpm) {
    return Commands.run(() -> startShooter(toprpm, bottomrpm))
        .until(() -> bothAtSpeed());
  }

  public void startShooter(double rpm) {
    topCommandRPM = rpm;
    bottomCommandRPM = rpm;
    setRunShooter();
  }

  public void startShooter(double toprpm, double bottomrpm) {
    topCommandRPM = toprpm;
    bottomCommandRPM = bottomrpm;
    setRunShooter();
  }

  public void setRunShooter() {
    runShooterVel = true;
  }

  public void resetRunShooter() {
    runShooterVel = false;
  }

  public boolean getRunShooter() {
    return runShooterVel;
  }

  public double getRPMTop() {
    if (RobotBase.isReal())
      return topRoller.getEncoder().getVelocity();
    else
      return topSimRPM;
  }

  public double getRPMBottom() {
    if (RobotBase.isReal())
      return bottomRoller.getEncoder().getVelocity();
    else
      return bottomSimRPM;
  }


  public void setCommandRPM(double rpm) {
    topCommandRPM = rpm;
    bottomCommandRPM = rpm;
  }


  public boolean topAtSpeed() {
    return true;
  }

  public boolean bottomAtSpeed() {
    return true;
  }

  public double getTopRPMError() {
    return topCommandRPM - getRPMTop();
  }

  public double getBottomRPMError() {
    return bottomCommandRPM - getRPMBottom();
  }

  public boolean bothAtSpeed() {
    shootersatspeed = topAtSpeed() && bottomAtSpeed();
    return shootersatspeed;
  }

  public double getTopAmps() {
    return topRoller.getOutputCurrent();
  }

  public double getBottomAmps() {
    return bottomRoller.getOutputCurrent();
  }

  public boolean getTopStickyFaults() {
    return topRoller.hasStickyFault();
  }

  public boolean getBottomStickyFaults() {
    return bottomRoller.hasStickyFault();
  }

  public Command clearStickyFaultsCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> topRoller.clearFaults()),
        runOnce(() -> bottomRoller.clearFaults()));
  }

  public void toggleTriggerSpkrShot() {
    okTriggerSpeakerShot = !okTriggerSpeakerShot;
  }

  public void toggleTriggerLobShot() {
    okTriggerLobShot = !okTriggerLobShot;
  }

  @Override
  public void simulationPeriodic() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/RPM", topCommandRPM);

    if (runShooterVel) {
      double toprpm = getTopCommandRPM();
      double bottomrpm = getBottomCommandRPM();
      if (RobotBase.isReal()) {
        topController.setReference(topSpeedLimiter.calculate(toprpm), ControlType.kVelocity);
        bottomController.setReference(bottomSpeedLimiter.calculate(bottomrpm), ControlType.kVelocity);
      }
    } else {
      stopMotors();
    }
    if (!topMotorConnected) {
      topMotorConnected = checkMotorCanOK(topRoller);
      SmartDashboard.putBoolean("Shooter//OKTShooter", topMotorConnected);
    }

    if (!bottomMotorConnected) {
      bottomMotorConnected = checkMotorCanOK(bottomRoller);
      SmartDashboard.putBoolean("Shooter//OKBShooter", bottomMotorConnected);
    }

  }

  private boolean checkMotorCanOK(SparkMax motor) {

    return RobotBase.isSimulation();
  }

  public Command testCan() {
    return Commands.parallel(
        Commands.runOnce(() -> topMotorConnected = false),
        runOnce(() -> bottomMotorConnected = false));
  }

  private double getTopCommandRPM() {
    return topCommandRPM;
  }

  private double getBottomCommandRPM() {
    return bottomCommandRPM;
  }


  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> {
            bottomRoller.setVoltage(volts.in(Volts));
            topRoller.setVoltage(volts.in(Volts));
          },
          log -> {
            log.motor("Top")
                .voltage(Volts.of(topRoller.getAppliedOutput() * topRoller.getBusVoltage()))
                .angularVelocity(Rotations.per(Minute).of(topRoller.getEncoder().getVelocity()))
                .angularPosition(Rotations.of(topRoller.getEncoder().getPosition()));
            // log.motor("Bottom")
            // .voltage(Volts.of(bottomRoller.getAppliedOutput() *
            // bottomRoller.getBusVoltage()))
            // .angularVelocity(Rotations.per(Minute).of(bottomEncoder.getVelocity()))
            // .angularPosition(Rotations.of(bottomEncoder.getPosition()));
          },
          this));

  public Command quasistaticForward() {
    return sysIdRoutine.quasistatic(Direction.kForward);
  }

  public Command quasistaticBackward() {
    return sysIdRoutine.quasistatic(Direction.kReverse);
  }

  public Command dynamicForward() {
    return sysIdRoutine.dynamic(Direction.kForward);
  }

  public Command dynamicBackward() {
    return sysIdRoutine.dynamic(Direction.kReverse);
  }

}
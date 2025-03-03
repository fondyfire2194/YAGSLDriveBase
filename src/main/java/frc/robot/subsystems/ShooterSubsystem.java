// Copytop (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {

  public SparkMax topRoller;
  public SparkClosedLoopController topController;

  public SparkMax bottomRoller;

  public SparkClosedLoopController bottomController;

  private SparkMaxConfig topConfig;
  private SparkMaxConfig bottomConfig;

  public AngularVelocity topCommandRPM = RPM.of(500);
  public AngularVelocity bottomCommandRPM = RPM.of(500);

  private AngularVelocity topSimRPM = RPM.of(0);
  private AngularVelocity bottomSimRPM = RPM.of(0);

  private boolean shootersatspeed;

  public double shootertolerancepct = 10;

  private boolean runShooterVel;

  private SlewRateLimiter topSpeedLimiter = new SlewRateLimiter(15000);
  private SlewRateLimiter bottomSpeedLimiter = new SlewRateLimiter(15000);
  public boolean topMotorConnected;
  public boolean bottomMotorConnected;

  public boolean okTriggerLobShot;

  public boolean okTriggerSpeakerShot;

  public final double topShooterKP = 3e-4;
  public final double topShooterKI = 0;
  public final double topShooterKD = 0.02;
  public final double topShooterKFF = 1.0 / 5700;
  public final double bottomShooterKP = 3e-4;
  public final double bottomShooterKI = 0;
  public final double bottomShooterKD = .02;
  public final double bottomShooterKFF = 1.0 / 5700;
  public final double voltageComp = 12;

   private MedianFilter topAmpFilter;
  private MedianFilter bottomAmpFilter;
  private MedianFilter topRPMFilter;
  private MedianFilter bottomRPMFilter;

  public final AngularVelocity maxShooterMotorRPM = RPM.of(5700);
  public final AngularVelocity minShooterMotorRPM = RPM.of(1500);

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

    
    topAmpFilter = new MedianFilter(20);
    bottomAmpFilter = new MedianFilter(20);
    topRPMFilter = new MedianFilter(20);
    bottomRPMFilter = new MedianFilter(20);
  }

  public void stopMotors() {
    runShooterVel = false;
    // topController.setReference(0, ControlType.kVelocity);
    bottomController.setReference(0, ControlType.kVelocity);
    topRoller.stopMotor();
    bottomRoller.stopMotor();
    topSpeedLimiter.reset(0);
    bottomSpeedLimiter.reset(0);
    topSimRPM = RPM.of(0);
    bottomSimRPM = RPM.of(0);
  }

  public Command stopShooterCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> stopMotors()),
        Commands.runOnce(() -> topCommandRPM = RPM.of(500)),
        Commands.runOnce(() -> bottomCommandRPM = RPM.of(500)));

  }

  public void setRPMTolerancePCT(double pct) {
    if (pct < 10 || pct > 25)
      pct = 10;
    shootertolerancepct = pct;
  }

  public void setRPMTolerancePCT() {
    shootertolerancepct = 10;
  }

  public Command startShooterCommand(AngularVelocity rpm) {
    return Commands.run(() -> startShooter(rpm))
        .until(() -> bothAtSpeed());
  }

  public Command startShooterCommand(AngularVelocity toprpm, AngularVelocity bottomrpm) {
    return Commands.run(() -> startShooter(toprpm, bottomrpm))
        .until(() -> bothAtSpeed());
  }

  public void startShooter(AngularVelocity rpm) {
    topCommandRPM = rpm;
    bottomCommandRPM = rpm;
    setRunShooter();
  }

  public void startShooter(AngularVelocity toprpm, AngularVelocity bottomrpm) {
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

  public AngularVelocity getRPMTop() {
    if (RobotBase.isReal())
      return RPM.of(topRoller.getEncoder().getVelocity());
    else
      return topSimRPM;
  }

  public AngularVelocity getRPMBottom() {
    if (RobotBase.isReal())
      return RPM.of(bottomRoller.getEncoder().getVelocity());
    else
      return bottomSimRPM;
  }

  public void setCommandRPM(AngularVelocity rpm) {
    topCommandRPM = rpm;
    bottomCommandRPM = rpm;
  }

  public void increaseShooterRPM(AngularVelocity val) {
    topCommandRPM.plus(val);
    bottomCommandRPM = topCommandRPM;
    if (topCommandRPM.gt(maxShooterMotorRPM))
      topCommandRPM = maxShooterMotorRPM;
  }

  public Command increaseRPMCommand(AngularVelocity val) {
    return Commands.runOnce(() -> increaseShooterRPM(val));
  }

  public void decreaseShooterRPM(AngularVelocity val) {
    topCommandRPM.minus(val);
    bottomCommandRPM = topCommandRPM;
    if (topCommandRPM.lt(minShooterMotorRPM))
      topCommandRPM = minShooterMotorRPM;
  }

  public Command decreaseRPMCommand(AngularVelocity val) {
    return Commands.runOnce(() -> decreaseShooterRPM(val));
  }

  public boolean topAtSpeed() {
    return !topCommandRPM.equals(0) &&
        Math.abs(topCommandRPM.in(RPM) - (getRPMTop().in(RPM))) < topCommandRPM.in(RPM) * shootertolerancepct / 100;
  }

  public boolean bottomAtSpeed() {
    return !bottomCommandRPM.equals(0)
        && Math.abs(bottomCommandRPM.in(RPM) - (getRPMBottom().in(RPM))) < bottomCommandRPM.in(RPM)
            * shootertolerancepct / 100;
  }

  public AngularVelocity getTopRPMError() {
    return topCommandRPM.minus(getRPMTop());
  }

  public AngularVelocity getBottomRPMError() {
    return bottomCommandRPM.minus(getRPMBottom());
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
    SmartDashboard.putNumber("Shooter/RPM", topCommandRPM.in(RPM));

    if (runShooterVel) {
      AngularVelocity toprpm = getTopCommandRPM();
      AngularVelocity bottomrpm = getBottomCommandRPM();
      if (RobotBase.isReal()) {
        topController.setReference(topSpeedLimiter.calculate(toprpm.in(RPM)), ControlType.kVelocity);
        bottomController.setReference(bottomSpeedLimiter.calculate(bottomrpm.in(RPM)), ControlType.kVelocity);
      }
    } else {
      stopMotors();
    }

    double topAmpFiltered = topAmpFilter.calculate(getTopAmps());
    double bottomAmpFiltered = bottomAmpFilter.calculate(getBottomAmps());
    double topRPMFiltered = topRPMFilter.calculate(getRPMTop().magnitude());
    double bottomRPMFiltered = bottomRPMFilter.calculate(getRPMBottom().magnitude());

    SmartDashboard.putNumber("Shooter/Top Amps", getTopAmps());
    SmartDashboard.putNumber("Shooter/Top Filtered Amps", topAmpFiltered);
    SmartDashboard.putNumber("Shooter/Bottom Amps", getBottomAmps());
    SmartDashboard.putNumber("Shooter/Bottom Filtered Amps", bottomAmpFiltered);

    SmartDashboard.putNumber("Shooter/Top RPM", getRPMTop().magnitude());
    SmartDashboard.putNumber("Shooter/Top Filtered RPM", topRPMFiltered);
    SmartDashboard.putNumber("Shooter/Bottom RPM", getRPMBottom().magnitude());
    SmartDashboard.putNumber("Shooter/Bottom Filtered RPM", bottomRPMFiltered);

  }


  private AngularVelocity getTopCommandRPM() {
    return topCommandRPM;
  }

  private AngularVelocity getBottomCommandRPM() {
    return bottomCommandRPM;
  }


}
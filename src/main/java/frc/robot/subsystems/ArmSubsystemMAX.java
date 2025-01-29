package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.SD;
import frc.robot.utils.TuneSparkPID;
import monologue.Annotations.Log;
import monologue.Logged;

public class ArmSubsystemMAX extends SubsystemBase implements Logged {

    public final SparkMax armMotor = new SparkMax(20, MotorType.kBrushless);

    public final SparkClosedLoopController armClosedLoopController;

    SparkMaxConfig armConfig;

    private TuneSparkPID tuneSparkPID;

    public ArmFeedforward armfeedforward;

    public boolean armMotorConnected;

    public double appliedOutput;

    public boolean inIZone;

    public double armVolts;

    public double appliedVolts;

    public Angle angleToleranceRads = Radians.of(Units.degreesToRadians(1));

    public boolean enableArm;

    public boolean presetOnce;

    public Angle simAngleRads = Radians.of(.001);

    public ProfiledPIDController armController;

    public TrapezoidProfile.Constraints constraints;

    private AngularVelocity maxVel = DegreesPerSecond.of(10);
    private AngularAcceleration maxAccel = DegreesPerSecondPerSecond.of(15);
    public final Angle armAngleOnBottomStopBar = Radians.of(Units.degreesToRadians(20));//
    public Angle armMin = Degrees.of(21);
    public Angle armMax = Degree.of(60);

    public Angle maxAngle = armMax;
    public Angle minAngle = armMin;
    private final MutAngle m_angle = Degrees.mutable(0);
    public double kp = 0;
    public double ki = 0;
    public double kd = 0;

    public final double armKg = 0.2;
    public final double armKs = 0.31;
    public final double armKv = 2;// volts per deg per sec so 12/max = 12/5=2.4
    public final double armKa = 0;

    public final double armKp = 30;
    public final double armKi = 0.5;
    public final double armKd = 0.0;

    public double armCurrentTarget;
    public double kpMax = .01;

    public final double NET_GEAR_RATIO = 200;// 100:1 then 2:1

    public final double DEGREES_PER_ENCODER_REV = 360 / NET_GEAR_RATIO;// 1.8

    public final double RADIANS_PER_ENCODER_REV = Units.degreesToRadians(DEGREES_PER_ENCODER_REV);// .0314

    public final double armConversionPositionFactor = RADIANS_PER_ENCODER_REV;

    public final double armConversionVelocityFactor = armConversionPositionFactor / 60; //

    double ks = armKs;
    double kv = armKv;
    double kg = armKg;
    double ka = armKa;
    // double lastkv = kv;
    boolean tuning = true;

    private int inPositionCtr;

    private double lastVel;

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
    private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 1, -90));
    private final MechanismLigament2d m_arm = m_armPivot.append(
            new MechanismLigament2d(
                    "Arm",
                    30,
                    getAngle().in(Degrees),
                    // Units.radiansToDegrees(simAngleRads.baseUnitMagnitude()),
                    6,
                    new Color8Bit(Color.kYellow)));

    private final MechanismLigament2d m_armTarget;
    private final TimeOfFlight m_rangeSensorLeft = new TimeOfFlight(31);
    private final TimeOfFlight m_rangeSensorRight = new TimeOfFlight(32);

    public ArmSubsystemMAX() {

        armClosedLoopController = armMotor.getClosedLoopController();
        armConfig = new SparkMaxConfig();

        armConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);

        armConfig.encoder
                .positionConversionFactor(armConversionPositionFactor)
                .velocityConversionFactor(armConversionVelocityFactor);

        armConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(kpMax)
                .outputRange(-1, 1).maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(1)
                .maxAcceleration(2)
                .allowedClosedLoopError(0.05);

        armConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        armConfig.signals.primaryEncoderPositionPeriodMs(5);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        constraints = new TrapezoidProfile.Constraints(maxVel.in(RadiansPerSecond),
                maxAccel.in(RadiansPerSecondPerSecond));

        armController = new ProfiledPIDController(.0001, 0, 0, constraints);

        tuneSparkPID = new TuneSparkPID("Arm", armMotor);

        m_armTarget = m_armPivot.append(new MechanismLigament2d(
                "ArmTarget", 30, getCurrentGoal().in(Degrees), 6, new Color8Bit(Color.kRed)));

        armfeedforward = new ArmFeedforward(ks, kg, kv);

        if (RobotBase.isReal()) {
            armMotor.getEncoder()
                    // .setPosition(0);
                    .setPosition(minAngle.in(Radians));
            armController.setGoal(minAngle.in(Radians));

        } else {
            simAngleRads = Radians.of(1);
        }

        armController.reset(getAngle().in(Radians));
        // setKp();

        SmartDashboard.putData("Arm//Arm Sim", m_mech2d);
        m_armTower.setColor(new Color8Bit(Color.kBlue));

        // Configure time of flight sensor for short ranging mode and sample
        // distance every 40 ms
        m_rangeSensorLeft.setRangingMode(RangingMode.Medium, 32);
        m_rangeSensorRight.setRangingMode(RangingMode.Medium, 32);

    }

    @Override
    public void periodic() {

        if (tuning)
            tuneSparkPID.tune();

        SD.sd2("Arm/Position", getAngle().in(Degrees));
        SD.sd2("Arm/Goal", getCurrentGoal().in(Degrees));
        SD.sd2("Arm/EncPosition", armMotor.getEncoder().getPosition());

        SD.sd2("Arm/FDistanceLeft", m_rangeSensorLeft.getRange());

        SD.sd("Arm/TOFDistanceSigmaLeft",m_rangeSensorLeft.getRangeSigma());

        SmartDashboard.putString("Arm/TOFStatusLeft", m_rangeSensorLeft.getStatus().toString());


        
        SD.sd2("Arm/FDistanceRight", m_rangeSensorRight.getRange());

        SD.sd("Arm/TOFDistanceSigmaRight",m_rangeSensorRight.getRangeSigma());

        SmartDashboard.putString("Arm/TOFStatusRight", m_rangeSensorRight.getStatus().toString());

    }

    @Override

    public void simulationPeriodic() {

        Angle a = getCurrentGoal().minus(simAngleRads);

        SD.sd2("Error", a.baseUnitMagnitude());

        SD.sd2("Simrads", simAngleRads.baseUnitMagnitude());

        if (a != Angle.ofBaseUnits(0, Radians)) {
            simAngleRads = simAngleRads.plus(a.div(10));
            SD.sd2("ErrorDiv", a.div(10).baseUnitMagnitude());

        }
        // Update the Mechanism Arm angle based on the simulated arm angle
        m_arm.setAngle(getAngle().in(Degrees));
        m_armTarget.setAngle(getAngle().in(Degrees));
    }

    public void setGoal(Angle angle) {
        armController.reset(getAngle().in(Radians));
        armController.setGoal(angle.in(Radians));
        inPositionCtr = 0;
    }

    @Log
    public boolean atGoal() {
        return armController.atGoal() && inPositionCtr == 3;
    }

    public void resetEncoder(double val) {
        armMotor.getEncoder().setPosition(val);
    }

    public void positionMAX() {
        double ff = armfeedforward.calculate(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());

        armClosedLoopController.setReference(
                armCurrentTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ff);

    }

    public void setTolerance(Angle toleranceRads) {
        angleToleranceRads = toleranceRads;
    }

    public void setTarget(Angle angle) {
        if (angle.gt(maxAngle))
            angle = maxAngle;
        if (angle.lt(minAngle))
            angle = minAngle;
        setGoal(angle);
    }

    // provides an always down final move to position by first moving
    // UDA parameter rads above goal if target is above actual
    public Command setGoalCommand(Angle angle) {
        return Commands.sequence(
                Commands.runOnce(() -> setTolerance(angleToleranceRads)),
                Commands.runOnce(() -> setGoal(angle)));

    }

    public void incrementArmAngle(Angle incAngle) {
        double temp = getCurrentGoal().in(Degrees);
        temp += incAngle.in(Degrees);
        if (temp > (maxAngle.in(Degrees)))
            temp = maxAngle.in(Degrees);
        Angle a = Degrees.of(temp);
        setGoal(a);
    }

    public void decrementArmAngle(Angle decAngle) {
        double temp = getCurrentGoal().in(Degrees);
        temp -= decAngle.in(Degrees);
        if (temp < (minAngle.in(Degrees)))
            temp = minAngle.in(Degrees);
        Angle a = Degrees.of(temp);
        setGoal(a);
    }

    public double getMotorEncoderAngleRadians() {
        return armMotor.getEncoder().getPosition();
    }

    @Log
    public double getMotorDegrees() {
        return Units.radiansToDegrees(armMotor.getEncoder().getPosition());
    }

    public double getMotorEncoderRadsPerSec() {
        return armMotor.getEncoder().getVelocity();
    }

    @Log
    public double getMotorEncoderDegsPerSec() {
        return Units.radiansToDegrees(armMotor.getEncoder().getVelocity());
    }

    @Log
    public Angle getCurrentGoal() {
        return Radians.of(armController.getGoal().position);
    }

    @Log
    public Angle getCurrentGoalDeg() {
        return getCurrentGoal();
    }

    @Log
    public Angle getAngle() {
        double rawAngle = armMotor.getEncoder().getPosition();
        double offsetAngle = rawAngle;
        m_angle.mut_replace(offsetAngle, Radians); // NOTE: the encoder must be configured with distancePerPulse in
                                                   // terms // // of radians
        return m_angle;
    }

    @Log
    public Angle getAngleError() {
        Angle diff = getCurrentGoal().minus(getAngle());
        return Radians.of(diff.in(Radians));
    }

    @Log
    public boolean getAtSetpoint() {
        return Math.abs(getAngleError().in(Radians)) <= (angleToleranceRads.in(Radians));
    }

    @Log
    public double getVoltsPerRadPerSec() {
        appliedVolts = armMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
        double temp = appliedVolts / getRadsPerSec();
        if (temp < 0 || temp > 10)
            temp = 0;
        return temp;
    }

    public double getRadsPerSec() {
        return armMotor.getEncoder().getVelocity();
    }

    public double getDegreesPerSec() {
        return Units.radiansToDegrees(armMotor.getEncoder().getVelocity());
    }

    public boolean onPlusSoftwareLimit() {

        return armMotor.getEncoder().getPosition() >= armMotor.configAccessor.softLimit.getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return armMotor.getEncoder().getPosition() <= armMotor.configAccessor.softLimit.getReverseSoftLimit();
    }

    public boolean onPlusHardwareLimit() {
        return armMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean onMinusHardwareLimit() {
        return armMotor.getReverseLimitSwitch().isPressed();
    }

    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() ||
                onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        armMotor.setVoltage(0);
    }

    public double getAmps() {
        return armMotor.getOutputCurrent();
    }

    public boolean isBraked() {
        return armMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return armMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || armMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public boolean getStickyFaults() {
        return armMotor.hasStickyFault();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> armMotor.clearFaults());
    }

    public Command testCan() {
        return Commands.runOnce(() -> armMotorConnected = false);
    }

}

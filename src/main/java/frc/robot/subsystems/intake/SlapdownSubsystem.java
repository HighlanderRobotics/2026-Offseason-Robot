package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SlapdownSubsystem extends SubsystemBase {
  public static final Rotation2d PIVOT_MIN_POSITION = Rotation2d.fromDegrees(0.0);
  // TODO:get pivot min position in degrees
  public static final Rotation2d PIVOT_MAX_POSITION = Rotation2d.fromDegrees(0);
  // TODO: pivot max position degrees
  public static final Rotation2d PIVOT_EXTENDED_POSITION = PIVOT_MIN_POSITION;
  public static final Rotation2d PIVOT_RETRACTED_POSITION = PIVOT_MAX_POSITION;
  public static final double CURRENT_ZEROING_THRESHOLD = 0.0;
  public static final double ROLLER_GEAR_RATIO = 3 / 1;
  public static final double PIVOT_GEAR_RATIO = 1.77777778 / 1;
  public static final double PIVOT_TO_CANCODER = 1 / 1.77777778;
  public static final double CANCODER_TO_PIVOT = 1.77777778 / 1;
  // TODO: implement true ratios for roller gear ratio, and pivot gear ratio,
  // pivot to cancoder and
  // cancoder to pivots
  // find zeroing threshold
  private final PivotIO pivotIO;
  private PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

  private final CANcoderIO cancoderIO;
  private CANcoderIOInputsAutoLogged cancoderIOInputs = new CANcoderIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  private Trigger atExtensionTrigger = new Trigger(this::atExtension).debounce(0.0);
  // TODO: find actual trigger debounce
  private LinearFilter currentFilter = LinearFilter.movingAverage(5);

  @AutoLogOutput(key = "Intake/Pivot/Current Filter Value")
  private double currentFilterValue = 0.0;

  // TODO: find actual filter value
  public SlapdownSubsystem(PivotIO pivotIO, CANcoderIO cancoderIO, RollerIO rollerIO) {
    this.pivotIO = pivotIO;
    this.cancoderIO = cancoderIO;
    this.rollerIO = rollerIO;
  }

  // TODO Auto-generated constructor stub

  public void slapdownInit() {
    pivotIO.resetEncoder(cancoderIOInputs.cancoderPositionRotations);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotIOInputs);
    Logger.processInputs("Intake/Pivot", pivotIOInputs);

    cancoderIO.updateInputs(cancoderIOInputs);
    Logger.processInputs("Intake/CANcoder", cancoderIOInputs);

    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Intake/Roller", rollerIOInputs);

    Logger.recordOutput("Intake/Pivot/Setpoint", pivotIO.getSetpoint());

    currentFilterValue = currentFilter.calculate(pivotIOInputs.statorCurrentAmps);
  }

  public Command agitate() {
    return Commands.sequence(
            this.run(
                    () -> {
                      pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
                      rollerIO.setRollerVelocity(0.0);
                      // TODO: find roller velocity
                    })
                .until(atExtensionTrigger),
            this.run(
                    () -> {
                      pivotIO.setMotorPositionSetpoint(
                          PIVOT_EXTENDED_POSITION.plus(Rotation2d.fromDegrees(0.0)));
                      rollerIO.setRollerVelocity(0.0);
                      // TODO: find pivot offset and roller velocity
                    })
                .until(atExtensionTrigger))
        .repeatedly();
  }

  public Command intake() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION, 0.0);
          rollerIO.setRollerVelocity(0.0);
          // TODO: find pivotio feed forward volts, and roller velocity
        });
  }

  public Command outtake() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
          rollerIO.setRollerVelocity(0.0);
          // TODO: set roller velocity
        });
  }

  public Command restExtended() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_EXTENDED_POSITION);
          rollerIO.setRollerVoltage(0.0);
        });
  }

  public Command restRetracted() {
    return this.run(
        () -> {
          pivotIO.setMotorPositionSetpoint(PIVOT_RETRACTED_POSITION);
          rollerIO.setRollerVoltage(0.0);
          // TODO: set roller voltage
        });
  }

  public Command runCurrentZeroing() {
    return Commands.sequence(
        this.run(() -> pivotIO.setMotorVoltage(0.0)), // TODO: set motor voltage
        Commands.waitUntil(() -> currentFilterValue > CURRENT_ZEROING_THRESHOLD),
        this.runOnce(() -> pivotIO.resetEncoder(PIVOT_MIN_POSITION)),
        Commands.print("Intake pivot zeroed"));
  }

  public Command zeroPivotOffCancoder() {
    return this.runOnce(() -> pivotIO.resetEncoder(cancoderIOInputs.cancoderPositionRotations));
  }

  public Rotation2d getPosition() {
    return pivotIOInputs.position;
  }

  public Rotation2d getPositionSetpoint() {
    return pivotIO.getSetpoint();
  }

  public boolean atExtension() {
    return MathUtil.isNear(getPositionSetpoint().getDegrees(), getPosition().getDegrees(), 10.0);
    // TODO: set .isNear tolerance
  }

  public static TalonFXConfiguration getPivotConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = 0;
    // TODO: set feedback remote sensorID
    config.Feedback.RotorToSensorRatio = CANCODER_TO_PIVOT;

    config.Feedback.SensorToMechanismRatio = 0;
    // TODO: set sensor to mech ratio

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.GravityArmPositionOffset = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;
    // TODO: set kS, kV, kS, kG, GravityArmPositionOffset, kP, kD

    config.CurrentLimits.StatorCurrentLimit = 30.0;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // TODO: set stator current limit, stator current lim enable, supply current
    // lim, supply current
    // lim enable

    config.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    config.MotionMagic.MotionMagicAcceleration = 0.0;
    // TODO: set cruise velocity, and acceleration

    return config;
  }

  public static TalonFXConfiguration getRollerConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // TODO: clockwise postitive

    config.Feedback.SensorToMechanismRatio = ROLLER_GEAR_RATIO;

    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.Slot0.kP = 0.0;
    config.Slot0.kD = 0.0;
    // TODO: set kS, kV, kA, kP, kD

    config.CurrentLimits.StatorCurrentLimit = 20.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 0.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // TODO: set stator current lim, stator current lim enable, supply current lim,
    // supply current
    // lim enable

    return config;
  }

  public static CANcoderConfiguration getCancoderConfig() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = 0.0;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;
    // TODO: set magnet offset, abs sensor discontinuity point

    return config;
  }
}

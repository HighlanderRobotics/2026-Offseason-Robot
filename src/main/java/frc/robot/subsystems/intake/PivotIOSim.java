package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim extends PivotIO {
  private final SingleJointedArmSim slapdownSim;
  private TalonFXSimState talonSim;
  private double lastLoopTime = 0.0;
  // TODI: find last loop time
  private final Notifier notifier;

  public PivotIOSim(int motorID, TalonFXConfiguration config, CANBus canbus) {

    super(motorID, config, canbus);

    slapdownSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX44Foc(1),
            SlapdownSubsystem.PIVOT_GEAR_RATIO,
            0.51536413,
            Units.inchesToMeters(13.854770),
            SlapdownSubsystem.PIVOT_MIN_POSITION.getRadians(),
            SlapdownSubsystem.PIVOT_MAX_POSITION.getRadians(),
            true,
            SlapdownSubsystem.PIVOT_RETRACTED_POSITION.getRadians());
    talonSim = motor.getSimState();
    talonSim.setMotorType(MotorType.KrakenX44);

    notifier =
        new Notifier(
            () -> {
              double deltaTime = (Utils.getCurrentTimeSeconds() - lastLoopTime);
              lastLoopTime = Utils.getCurrentTimeSeconds();
              talonSim.setSupplyVoltage(RobotController.getBatteryVoltage());
              slapdownSim.setInputVoltage(talonSim.getMotorVoltage());
              slapdownSim.update(deltaTime);

              double positionRotations =
                  Units.radiansToRotations(
                      slapdownSim.getAngleRads() * SlapdownSubsystem.PIVOT_GEAR_RATIO);
              talonSim.setRawRotorPosition(positionRotations);

              double velocityRPS =
                  Units.radiansToRotations(
                      slapdownSim.getAngleRads() * SlapdownSubsystem.PIVOT_GEAR_RATIO);
              talonSim.setRotorVelocity(velocityRPS);
            });
    notifier.startPeriodic(0.002);
  }
}

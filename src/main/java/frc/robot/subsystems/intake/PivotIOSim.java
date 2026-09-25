package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
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

  public PivotIOSim(
      int motorID,
      TalonFXConfiguration config,
      SingleJointedArmSim intakeSim,
      MotorType motorType,
      CANBus canbus) {

    super(motorID, config, canbus);

    slapdownSim = intakeSim;
    talonSim = motor.getSimState();
    talonSim.setMotorType(motorType);

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

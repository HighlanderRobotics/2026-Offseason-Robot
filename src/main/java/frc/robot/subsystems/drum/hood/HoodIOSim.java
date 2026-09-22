package frc.robot.subsystems.drum.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.drum.DrumSubsystem;

public class HoodIOSim extends HoodIO {
  private TalonFXSimState simState;
  private SingleJointedArmSim physicsSim;

  private double lastLoopTime = 0.0;
  private final double simLoopPeriod = 0.02;

  private Notifier notifier;

  public HoodIOSim(CANBus canBus) {
    super(canBus);

    simState = motor.getSimState();

    // TODO: VALUES FROM CAD
    physicsSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1), // TODO: IS IT X44?
            DrumSubsystem.HOOD_GEAR_RATIO,
            0.111666,
            Units.inchesToMeters(8.800269),
            Units.degreesToRadians(10),
            Units.degreesToRadians(45),
            true,
            Units.degreesToRadians(10));

    notifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastLoopTime;
              lastLoopTime = currentTime;

              simState.setSupplyVoltage(RobotController.getBatteryVoltage());

              physicsSim.setInputVoltage(simState.getMotorVoltage());
              physicsSim.update(deltaTime);

              simState.setRawRotorPosition(
                  Units.radiansToRotations(
                      physicsSim.getAngleRads() * DrumSubsystem.HOOD_GEAR_RATIO));
              simState.setRotorVelocity(
                  Units.radiansToRotations(
                      physicsSim.getVelocityRadPerSec() * DrumSubsystem.HOOD_GEAR_RATIO));
            });

    notifier.startPeriodic(simLoopPeriod);
  }
}

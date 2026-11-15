package frc.robot.subsystems.drum.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drum.DrumSubsystem;

public class FlywheelIOSim extends FlywheelIO {

  private TalonFXSimState simState;
  private TalonFXSimState followerSimState1;
  private TalonFXSimState followerSimState2;
  private TalonFXSimState followerSimState3;


  private DCMotorSim physicsSim;

  private Notifier simNotifier;

  private final double simLoopPeriod = 0.02;
  private double lastSimTime = 0.0;
  private TalonFXSimState[] followerSimStates = new TalonFXSimState[3];
  public FlywheelIOSim(CANBus canBus) {
    super(canBus);

    simState = leader.getSimState();
    followerSimState1 = follower1.getSimState();
    followerSimState2 = follower2.getSimState();
    followerSimState3 = follower3.getSimState();
    followerSimStates[0] = followerSimState1;
    followerSimStates[1] = followerSimState2;
    followerSimStates[2] = followerSimState3;
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(4), 0.002407, DrumSubsystem.FLYWHEEL_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(4));

    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              simState.setSupplyVoltage(RobotController.getBatteryVoltage());
              
              physicsSim.setInputVoltage(simState.getMotorVoltage());
              physicsSim.update(deltaTime);

              simState.setRawRotorPosition(
                  physicsSim.getAngularPositionRotations() * physicsSim.getGearing());
              simState.setRotorVelocity(
                  physicsSim.getAngularVelocity().in(RotationsPerSecond) * physicsSim.getGearing());
              for (int i=0; i<followerSimStates.length; i++) {
                followerSimStates[i].setSupplyVoltage(RobotController.getBatteryVoltage());
                 followerSimStates[i].setRawRotorPosition(
                  physicsSim.getAngularPositionRotations() * physicsSim.getGearing());
              followerSimStates[i].setRotorVelocity(
                  physicsSim.getAngularVelocity().in(RotationsPerSecond) * physicsSim.getGearing());
              }
            });

    simNotifier.setName("Flywheel");
    simNotifier.startPeriodic(simLoopPeriod);
  }
}

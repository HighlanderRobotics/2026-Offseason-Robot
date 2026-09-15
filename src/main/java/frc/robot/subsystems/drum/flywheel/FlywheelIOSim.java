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
    private DCMotorSim physicsSim;

    private Notifier simNotifier;

    private final double simLoopPeriod = 0.02;
    private double lastSimTime = 0.0;

    public FlywheelIOSim(CANBus canBus) {
        super(canBus);

        simState = leader.getSimState();
        physicsSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(4),
                0.01,
                DrumSubsystem.FLYWHEEL_GEAR_RATIO
            ),
            DCMotor.getKrakenX60Foc(4)
        );

        simNotifier = new Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            simState.setSupplyVoltage(RobotController.getBatteryVoltage());

            physicsSim.setInputVoltage(simState.getMotorVoltage());
            physicsSim.update(deltaTime);

            simState.setRawRotorPosition(physicsSim.getAngularPositionRotations() * physicsSim.getGearing());
            simState.setRotorVelocity(physicsSim.getAngularVelocity().in(RotationsPerSecond) * physicsSim.getGearing());
        });

        simNotifier.startPeriodic(simLoopPeriod);
    }
    
}

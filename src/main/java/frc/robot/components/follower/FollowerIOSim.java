package frc.robot.components.follower;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class FollowerIOSim extends FollowerIO {
    private TalonFXSimState simState;

    private final DoubleSupplier leaderPositionSupplier;
    private final DoubleSupplier leaderVelSupplier;

    public FollowerIOSim(int motorID, int leaderID, MotorAlignmentValue alignment, CANBus canBus, DoubleSupplier leaderPositionSupplier, DoubleSupplier leaderVelSupplier) {
        super(motorID, leaderID, alignment, canBus);

        // TODO: I'M NOT EVEN SURE THIS WORKS
        this.leaderPositionSupplier = leaderPositionSupplier;
        this.leaderVelSupplier = leaderVelSupplier;
    }

    @Override
    public void updateInputs(FollowerIOInputs inputs) {
        // First update sim state with the mech pos and vel
        simState.setRawRotorPosition(leaderPositionSupplier.getAsDouble());
        simState.setRotorVelocity(leaderVelSupplier.getAsDouble());

        super.updateInputs(inputs);
    }
}

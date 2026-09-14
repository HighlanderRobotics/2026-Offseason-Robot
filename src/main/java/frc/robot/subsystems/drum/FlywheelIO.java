package frc.robot.subsystems.drum;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        public double velocityRotPerSec = 0.0;
        public double voltage = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempC = 0.0;
        // For sysid
        public double flywheelPositionRotations = 0.0;
    }

    public final TalonFX leader;

    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<Voltage> voltage;
    private StatusSignal<Current> statorCurrent;
    private StatusSignal<Current> supplyCurrent;
    private StatusSignal<Temperature> temp;
    private StatusSignal<Angle> flywheelPosition;

    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
    private MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);

    private double velocitySetpointRotPerSec = 0.0;

    public FlywheelIO(CANBus canBus) {
        // TODO: CORRECT ID
        leader = new TalonFX(0, canBus);

        velocity = leader.getVelocity();
        voltage = leader.getMotorVoltage();
        statorCurrent = leader.getStatorCurrent();
        supplyCurrent  = leader.getSupplyCurrent();
        temp  = leader.getDeviceTemp();
        flywheelPosition  = leader.getPosition();

        // TODO: MAYBE INCREASE FREQUENCY FOR VOLTAGE
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            velocity,
            voltage,
            statorCurrent,
            supplyCurrent,
            temp,
            flywheelPosition
        );
        leader.optimizeBusUtilization();
    }

    public void updateInputs(FlywheelIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            velocity,
            voltage,
            statorCurrent,
            supplyCurrent,
            temp,
            flywheelPosition
        );
        inputs.flywheelPositionRotations = flywheelPosition.getValue().in(Rotation);
        inputs.velocityRotPerSec = velocity.getValue().in(Rotation.per(Second));
        inputs.voltage = voltage.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.tempC = temp.getValue().in(Celsius);
    }

    public void setVoltage(double voltage) {
        leader.setControl(voltageOut.withOutput(voltage));
    }

    // TODO: ADD ACCEL CONTROL
    public void setVelocitySetpoint(double velocityRotPerSec) {
        velocitySetpointRotPerSec = velocityRotPerSec;
        leader.setControl(motionMagic.withVelocity(velocityRotPerSec));
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/** Things to log
temp
voltage
connected?
stator amps
supply amps
position for rots
velocity rots
 */
public class IndexerIO {
    @AutoLog 
    public static class IndexerIOInputs {
        // main motor
        public double indexerVelocityRotPerSecond = 0.0;
        public double indexerindexerPositionRots = 0.0;
        public double indexerStatorAmps = 0.0;
        public double indexerSupplyAmps = 0.0;
        public double indexerTempatureCelcius = 0.0;
        public double indexerVoltage = 0.0;
        public boolean indexerConnected = false; // i think it updates in the updateinputs

        // kicker leader
        public double kickerLeaderVelocityRotPerSecond = 0.0;
        public double kickerLeaderPositionRots = 0.0;
        public double kickerLeaderStatorAmps = 0.0;
        public double kickerLeaderSupplyAmps = 0.0;
        public double kickerLeaderVoltage = 0.0;
        public double kickerLeaderTempatureCelcius = 0.0;
        public boolean kickerLeaderConnected = false;

        // kicker follower
        public double kickerFollowerVelocityRotPerSecond = 0.0;
        public double kickerFollowerPositionRots = 0.0;
        public double kickerFollowerStatorAmps = 0.0;
        public double kickerFollowerSupplyAmps = 0.0;
        public double kickerFollowerVoltage = 0.0;
        public double kickerFollowerTempatureCelcius = 0.0;
        public boolean kickerFollowerConnected = false;
    }
    // gear ratios
    public static final double INDEXER_GEAR_RATIO = 50.0/12.0;
    public static final double KICKER_GEAR_RATIO = 30.0/12.0;

    // make motors
    protected final TalonFX indexerMotor;
    protected final TalonFX kickerLeaderMotor;
    protected final TalonFX kickerFollowerMotor;

    // indexer status signals
    private final StatusSignal<AngularVelocity> indexerAngularVelocityRotsPerSec;
    private final StatusSignal<Angle> indexerPosition;
    private final StatusSignal<Current> indexerStatorCurrent;
    private final StatusSignal<Current> indexerSupplyCurrent;
    private final StatusSignal<Voltage> indexerVoltage;
    private final StatusSignal<Temperature> indexerTempature;

    // kicker leader status signals
    private final StatusSignal<AngularVelocity> kickerLeaderAngularVelocityRotsPerSec;
    private final StatusSignal<Angle> kickerLeaderPosition;
    private final StatusSignal<Current> kickerLeaderStatorCurrent;
    private final StatusSignal<Current> kickerLeaderSupplyCurrent;
    private final StatusSignal<Voltage> kickerLeaderVoltage;
    private final StatusSignal<Temperature> kickerLeaderTempature;

    // kicker follower status signals
    private final StatusSignal<AngularVelocity> kickerFollowerAngularVelocityRotsPerSec;
    private final StatusSignal<Angle> kickerFollowerPosition;
    private final StatusSignal<Current> kickerFollowerStatorCurrent;
    private final StatusSignal<Current> kickerFollowerSupplyCurrent;
    private final StatusSignal<Voltage> kickerFollowerVoltage;
    private final StatusSignal<Temperature> kickerFollowerTempature;

    // voltage outs
    private VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC((true));
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);
    
    public static TalonFXConfiguration getIndexerConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // stopping
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // direction

        config.Feedback.SensorToMechanismRatio = INDEXER_GEAR_RATIO;

        // pid values are copied from the main branch indexer file
        config.Slot0.kS = 0;
        config.Slot0.kG = 0;
        config.Slot0.kV = 0.49448;
        config.Slot0.kA = 0.032974;
        config.Slot0.kP = 0;
        config.Slot0.kD = 0;

        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        return config;

    }

    public static TalonFXConfiguration getKickerConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.Feedback.SensorToMechanismRatio = KICKER_GEAR_RATIO;

        // pid values from main branch indexer file
        config.Slot0.kS = 0.018992;
        config.Slot0.kG = 0;
        config.Slot0.kV = 0.30798;
        config.Slot0.kA = 0.022312;
        config.Slot0.kP = 0;
        config.Slot0.kD = 0;

        // limits
        config.CurrentLimits.StatorCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        return config;
    }

    public IndexerIO(CANBus canBus) {
        indexerMotor = new TalonFX(11, canBus);
        indexerMotor.getConfigurator().apply(IndexerIO.getIndexerConfiguration());

        kickerLeaderMotor = new TalonFX(12, canBus);
        kickerLeaderMotor.getConfigurator().apply(IndexerIO.getKickerConfiguration());
        
        kickerFollowerMotor = new TalonFX(13, canBus);
        kickerFollowerMotor.getConfigurator().apply(IndexerIO.getKickerConfiguration());

        // get data from motor

        indexerAngularVelocityRotsPerSec = indexerMotor.getVelocity();
        indexerPosition = indexerMotor.getPosition();
        indexerStatorCurrent = indexerMotor.getStatorCurrent();
        indexerSupplyCurrent = indexerMotor.getSupplyCurrent();
        indexerVoltage = indexerMotor.getMotorVoltage();
        indexerTempature = indexerMotor.getDeviceTemp();

        // im copy pasting because im not doing this 2 more times

        kickerLeaderAngularVelocityRotsPerSec = kickerLeaderMotor.getVelocity();
        kickerLeaderPosition = kickerLeaderMotor.getPosition();
        kickerLeaderStatorCurrent = kickerLeaderMotor.getStatorCurrent();
        kickerLeaderSupplyCurrent = kickerLeaderMotor.getSupplyCurrent();
        kickerLeaderVoltage = kickerLeaderMotor.getMotorVoltage();
        kickerLeaderTempature = kickerLeaderMotor.getDeviceTemp();

        kickerFollowerAngularVelocityRotsPerSec = kickerFollowerMotor.getVelocity();
        kickerFollowerPosition = kickerFollowerMotor.getPosition();
        kickerFollowerStatorCurrent = kickerFollowerMotor.getStatorCurrent();
        kickerFollowerSupplyCurrent = kickerFollowerMotor.getSupplyCurrent();
        kickerFollowerVoltage = kickerFollowerMotor.getMotorVoltage();
        kickerFollowerTempature = kickerFollowerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            indexerAngularVelocityRotsPerSec,
            indexerPosition,
            indexerStatorCurrent,
            indexerSupplyCurrent,
            indexerVoltage,
            indexerTempature,
            kickerLeaderAngularVelocityRotsPerSec,
            kickerLeaderPosition,
            kickerLeaderStatorCurrent,
            kickerLeaderSupplyCurrent,
            kickerLeaderVoltage,
            kickerLeaderTempature,
            kickerFollowerAngularVelocityRotsPerSec,
            kickerFollowerPosition,
            kickerFollowerStatorCurrent,
            kickerFollowerSupplyCurrent,
            kickerFollowerVoltage,
            kickerFollowerTempature
        );
        indexerMotor.optimizeBusUtilization();
        kickerLeaderMotor.optimizeBusUtilization();
        kickerFollowerMotor.optimizeBusUtilization();

        kickerFollowerMotor.setControl(new Follower(kickerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }
    public void setIndexerVoltage(Double voltage) {
        indexerMotor.setControl(voltageOut.withOutput(voltage));
    }
    public void setKickerControl(Double voltage) {
        kickerLeaderMotor.setControl(voltageOut.withOutput(voltage));
    }
    public void updateinputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            indexerAngularVelocityRotsPerSec,
            indexerPosition,
            indexerStatorCurrent,
            indexerSupplyCurrent,
            indexerVoltage,
            indexerTempature,
            kickerLeaderAngularVelocityRotsPerSec,
            kickerLeaderPosition,
            kickerLeaderStatorCurrent,
            kickerLeaderSupplyCurrent,
            kickerLeaderVoltage,
            kickerLeaderTempature,
            kickerFollowerAngularVelocityRotsPerSec,
            kickerFollowerPosition,
            kickerFollowerStatorCurrent,
            kickerFollowerSupplyCurrent,
            kickerFollowerVoltage,
            kickerFollowerTempature
        );
        // check if indexer connected
        inputs.indexerConnected = 
            BaseStatusSignal.isAllGood(
                indexerAngularVelocityRotsPerSec,
                indexerPosition,
                indexerStatorCurrent,
                indexerSupplyCurrent,
                indexerVoltage,
                indexerTempature
            );
        inputs.indexerVelocityRotPerSecond = indexerAngularVelocityRotsPerSec.getValueAsDouble();
        inputs.indexerindexerPositionRots = indexerPosition.getValueAsDouble();
        inputs.indexerStatorAmps = indexerStatorCurrent.getValueAsDouble();
        inputs.indexerSupplyAmps = indexerSupplyCurrent.getValueAsDouble();
        inputs.indexerVoltage = indexerVoltage.getValueAsDouble();
        inputs.indexerTempatureCelcius = indexerTempature.getValueAsDouble();

        // kicker leader
        inputs.kickerLeaderConnected = 
            BaseStatusSignal.isAllGood(
                kickerLeaderAngularVelocityRotsPerSec,
                kickerLeaderPosition,
                kickerLeaderStatorCurrent,
                kickerLeaderSupplyCurrent,
                kickerLeaderVoltage,
                kickerLeaderTempature
            );
        inputs.kickerLeaderVelocityRotPerSecond = kickerLeaderAngularVelocityRotsPerSec.getValueAsDouble();
        inputs.kickerLeaderPositionRots = kickerLeaderPosition.getValueAsDouble();
        inputs.kickerLeaderStatorAmps = kickerLeaderStatorCurrent.getValueAsDouble();
        inputs.kickerLeaderSupplyAmps = kickerLeaderSupplyCurrent.getValueAsDouble();
        inputs.kickerLeaderVoltage = kickerLeaderVoltage.getValueAsDouble();
        inputs.kickerLeaderTempatureCelcius = kickerLeaderTempature.getValueAsDouble();

        // kicker follower
        inputs.kickerFollowerConnected = 
            BaseStatusSignal.isAllGood(
                kickerFollowerAngularVelocityRotsPerSec,
                kickerFollowerPosition,
                kickerFollowerStatorCurrent,
                kickerFollowerSupplyCurrent,
                kickerFollowerVoltage,
                kickerFollowerTempature
            );
        inputs.kickerFollowerVelocityRotPerSecond = kickerFollowerAngularVelocityRotsPerSec.getValueAsDouble();
        inputs.kickerFollowerPositionRots = kickerFollowerPosition.getValueAsDouble();
        inputs.kickerFollowerStatorAmps = kickerFollowerStatorCurrent.getValueAsDouble();
        inputs.kickerFollowerSupplyAmps = kickerFollowerSupplyCurrent.getValueAsDouble();
        inputs.kickerFollowerVoltage = kickerFollowerVoltage.getValueAsDouble();
        inputs.kickerFollowerTempatureCelcius = kickerFollowerTempature.getValueAsDouble();
    }
}
 
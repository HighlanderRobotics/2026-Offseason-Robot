// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.*;

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
        public double indexerVelocityPerSecond = 0.0;
        public double indexerPositonRots = 0.0;
        public double indexerStatorAmps = 0.0;
        public double indexerSupplyAmps = 0.0;
        public double indexerTempature = 0.0;
    }
}

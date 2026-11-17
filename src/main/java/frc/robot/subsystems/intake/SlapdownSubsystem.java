// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlapdownSubsystem extends SubsystemBase {
  /** Creates a new SlapdownSubsystem. */
  public SlapdownSubsystem() {
    public static final Rotation2d PIVOT_MIN_POSITION = Rotation2d.fromDegrees(0.0);
    public static final Rotation2d PIVOT_MAX_POSITION = Rotation2d.fromDegrees(130.0);
    public static final Rotation2d PIVOT_EXTENDED_POSITION = PIVOT_MIN_POSITION.plus(Rotation2d.fromDegrees(3.0));
    public static final Rotation2d PIVOT_RETRACTED_POSITION = PIVOT_MAX_POSITION;
    public static final double CURRENT_ZEROING_THRESHOLD = 20.0;
    public static final double ROLLER_GEAR_RATIO = 36.0 / 12.0;
    public static final double PIVOT_GEAR_RATIO = (60.0 / 8.0) * (64.0 / 16.0);
    public static final double PIVOT_TO_CANCODER = 1.0 / 1.77777778;
    public static final double CANCODER_TO_PIVOT = 1.77777778 / 1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autos.Action;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final SwerveSubsystem swerve;
  private final AutoFactory factory;
  private static boolean autoIntake;

  @AutoLogOutput(key = "Superstructure/Auto Intake Request")
  public static Trigger autoIntakeReq =
      new Trigger(() -> autoIntake).and(DriverStation::isAutonomous);

  public enum Action {
    NOTHING
  }

  // testing please organize later :)
  public enum Path {
    LStartToFirstDip("StartToFirstDip", Action.NOTHING);
    private final String name;
    private final Action action;

    /**
     * @param name the name of the path in choreo. MUST match
     * @param action the action to perform during/at the end of the path
     */
    private Path(String name, Action action) {
      this.name = name;
      this.action = action;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      // AutoRoutine docs say that this "creates" a new trajectory, but the factory does check if
      // it's already present
      return routine.trajectory(name);
    }
  }

  public Autos(SwerveSubsystem swerve) {
    this.swerve = swerve;
    factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose,
            swerve.choreoDriveController(),
            true,
            swerve,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/Active Traj",
                  DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Blue)
                      ? traj.getPoses()
                      : traj.flipped().getPoses());
              Logger.recordOutput("Choreo/Active Traj Name", traj.name());
            });
  }

  public Command runPath(Path path, AutoRoutine routine) {
    Action action = path.action;
    switch (action) {
      case NOTHING:
        return emptyPath(path, routine);
      default:
        return Commands.none();
    }
  }

  public Command emptyPath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAllReqsFalse(),
        path.getTrajectory(routine).cmd().until(path.getTrajectory(routine).done()));
  }

  public Command startIntaking() {
    return Commands.runOnce(() -> autoIntake = true);
  }

  public Command stopIntaking() {
    return Commands.runOnce(() -> autoIntake = false);
  }

  public Command setAllReqsFalse() {
    return Commands.sequence(stopIntaking());
  }

  public void setAllReqsFalsenotcmd() {
    autoIntake = false;
  }

  public Command createAuto(
      String name, Path[] paths, Command setClimbSideCmd, Command... startingCommands) {
    final AutoRoutine routine = factory.newRoutine(name);

    Command autoCommand =
        paths[0]
            .getTrajectory(routine)
            .resetOdometry()
            .alongWith(setClimbSideCmd)
            .andThen(startingCommands);
    for (Path p : paths) {
      autoCommand = autoCommand.andThen(runPath(p, routine));
    }
    routine.active().onTrue(autoCommand);
    return routine.cmd();
  }
}

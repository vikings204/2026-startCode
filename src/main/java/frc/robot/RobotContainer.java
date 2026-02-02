package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Controller;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Gamepad;

import static frc.robot.Robot.AutoModeChooser;
import static frc.robot.Robot.ControlModeChooser;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final PoseEstimationSubsystem PoseEstimation = new PoseEstimationSubsystem(Swerve::getYaw, Swerve::getPositions);

    Gamepad DRIVER = new Gamepad(Controller.DRIVER_PORT);
    Gamepad OPERATOR = new Gamepad(Controller.OPERATOR_PORT);

    public RobotContainer() {
        ControlModeChooser.onChange((ControlMode mode) -> {
            if (mode == ControlMode.SINGLE) {
                OPERATOR = new Gamepad(Controller.DRIVER_PORT);
            } else {
                OPERATOR = new Gamepad(Controller.OPERATOR_PORT);
            }
            configureDefaultCommands();
            configureButtonBindings();
        });

        Shuffleboard.getTab("debug").add("swerve", Swerve);
        // Shuffleboard.getTab("main").add("shooter", Shooter);
        Shuffleboard.getTab("main").add("zero swerve", new RunCommand(Swerve::zeroGyro)).withWidget(BuiltInWidgets.kCommand);

        AutoBuilder.configure(
                PoseEstimation::getCurrentPose, // Robot pose supplier
                PoseEstimation::setCurrentPose,
                Swerve::getSpeeds,
                Swerve::driveRobotRelative,// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                Constants.Auto.PATH_FOLLOWER_CONFIG, // The path follower configuration
                Constants.Auto.config, // The robot configuration
                //() -> Robot.alliance == DriverStation.Alliance.Red,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                Swerve // Reference to this subsystem to set requirements
        );
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        Swerve.setDefaultCommand(
                new TeleopSwerveCommand(
                        Swerve,
                        () -> -1 * DRIVER.getLeftX(),
                        () -> 1 * DRIVER.getLeftY(),
                        () -> -1 * DRIVER.getRightX(),
                        () -> false,
                        () -> false,
                        () -> false));
    }


    private void configureButtonBindings() {

    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(AutoModeChooser.getSelected().pathplannerName);
    }

    public void checkAnalogs() {

    }
}

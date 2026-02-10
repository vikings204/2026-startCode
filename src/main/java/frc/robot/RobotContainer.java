package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Intake.Positions;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gamepad;

import static frc.robot.Robot.*;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final LEDSubsystem LED = new LEDSubsystem();
    public final IntakeSubsystem Intake = new IntakeSubsystem();
    public final ShooterSubsystem Shooter = new ShooterSubsystem();
    //public final ClimberSubsystem Climber = new ClimberSubsystem();
    public final PoseEstimationSubsystem PoseEstimation = new PoseEstimationSubsystem(Swerve::getYaw, Swerve::getPositions, Swerve::getSpeeds);

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
        Shuffleboard.getTab("debug").add("intake", Intake);
        Shuffleboard.getTab("debug").add("shooter", Shooter);
        //Shuffleboard.getTab("debug").add("climber", Climber);
        Shuffleboard.getTab("main").add("zero gyro", new RunCommand(Swerve::zeroGyro)).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("main").add("zero elevator", new RunCommand(Intake::zeroEncoders, Intake)).withWidget(BuiltInWidgets.kCommand);
        Shuffleboard.getTab("main").add("zero pose estimator", new RunCommand(PoseEstimation::resetPose, PoseEstimation)).withWidget(BuiltInWidgets.kCommand);

        AutoBuilder.configure(
            PoseEstimation::getPose, // Robot pose supplier
            PoseEstimation::setPose,
            Swerve::getSpeeds,
            (speeds, feedforwards) -> Swerve.driveRobotRelative(speeds),//byass need for PathFeedfowards with lamda, hacky solution idk if its good// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Constants.Auto.PATH_FOLLOWER_CONFIG, // The path follower configuration
            Constants.Auto.config, // The robot configuration
            () -> ALLIANCE == DriverStation.Alliance.Red,
            Swerve // Reference to this subsystem to set requirements
        );

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

        NamedCommands.registerCommand("ZERO", new InstantCommand(() -> Intake.setPosition(Positions.ZERO), Intake));
        NamedCommands.registerCommand("INTAKE", new InstantCommand(Intake::IntakeAUTO, Intake));
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> Shooter.shootMotor(true, 1), Shooter));
        NamedCommands.registerCommand("IntakeMotorON", new InstantCommand(() -> Intake.IntakeMotor(true), Intake));
        NamedCommands.registerCommand("IntakeMotorOFF", new InstantCommand(() -> Intake.IntakeMotor(false), Intake));

        NamedCommands.registerCommand("Intake_Elevator", new InstantCommand(() -> Intake.setPosition(Positions.INTAKE), Intake));
        NamedCommands.registerCommand("zeroGyro", new InstantCommand(Swerve::zeroGyro, Swerve));

        configureDefaultCommands();
        configureButtonBindings();
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto(AutoModeChooser.getSelected().pathplannerName);
    }

    private void configureDefaultCommands() {
        Swerve.setDefaultCommand(
                new TeleopSwerveCommand(
                        Swerve,
                        () -> -1 * DRIVER.getLeftY(),
                        () -> -1 * DRIVER.getLeftX(),
                        () -> -1 * DRIVER.getRightX(),
                        () -> false));


        //Elevator.setDefaultCommand(
        //        new RunCommand(
        //                () -> Elevator.jogPositive(false),
        //                Elevator));
        //Tongue.setDefaultCommand(new RunCommand(()->Tongue.readSensor()),Tongue);
    }


    private void configureButtonBindings() {

        //new JoystickButton(DRIVER, 3).onTrue(new RunCommand(Swerve::zeroGyro));

        //new JoystickButton(DRIVER, 8).onTrue(new InstantCommand(Swerve::setSpeed)); // slow mode (terrible implementation)


        new JoystickButton(OPERATOR, 7)
                .whileTrue(new InstantCommand(() -> Intake.jogPositive(true), Intake))
                .onFalse(new InstantCommand(() -> Intake.jogPositive(false), Intake));

        new JoystickButton(OPERATOR, 8)
                .whileTrue(new InstantCommand(() -> Intake.jogNegative(true), Intake))
                .onFalse(new InstantCommand(() -> Intake.jogNegative(false), Intake));


   /*      new JoystickButton(OPERATOR, 7)
                .whileTrue(new RunCommand(() -> Elevator.jogPositive(true), Elevator));

        new JoystickButton(OPERATOR, 8)
                .whileTrue(new RunCommand(() -> Elevator.jogNegative(true), Elevator));*/

        new JoystickButton(OPERATOR, 3)
                .onTrue(new RunCommand(() -> Intake.setPosition(Positions.ZERO), Intake));
        new JoystickButton(OPERATOR, 4)
                .onTrue(new RunCommand(() -> Intake.setPosition(Positions.INTAKE), Intake));

        //  new JoystickButton(DRIVER, 1).
        //         whileTrue(Swerve.driveToPose());
//        new JoystickButton(DRIVER, 1).whileTrue(new AlignCommand(false, Swerve));

        new JoystickButton(DRIVER, 4)
                //.whileTrue(StupidAlignLeft); // A is left
                .whileTrue(new InstantCommand(() -> Shooter.shootMotor(true, .25), Shooter))
                .onFalse(new InstantCommand(() -> Shooter.shootMotor(false, 0), Shooter));
        new JoystickButton(DRIVER, 1)
                //.whileTrue(StupidAlignRight); // B is right
                .whileTrue(new InstantCommand(() -> Shooter.shootMotor(true, .5), Shooter))
                .onFalse(new InstantCommand(() -> Shooter.shootMotor(false, 0), Shooter));
        new JoystickButton(DRIVER, 2)
                //.whileTrue(ColorAlignLeft);
                .whileTrue(new InstantCommand(() -> Shooter.shootMotor(true, .75), Shooter))
                .onFalse(new InstantCommand(() -> Shooter.shootMotor(false, 0), Shooter));
        new JoystickButton(DRIVER, 3)
                //.whileTrue(ColorAlignRight);
                .whileTrue(new InstantCommand(() -> Shooter.shootMotor(true, 1), Shooter))
                .onFalse(new InstantCommand(() -> Shooter.shootMotor(false, 0), Shooter));
    }

    public void checkAnalogs() {
        /*if (OPERATOR.getRightTriggerAxis() > .5) {
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));
        }*/

        /*if (OPERATOR.getLeftTriggerAxis() > .5) {
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));
        }*/

        /*if (OPERATOR.getRightY() < -.5) {
            CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.ShootArm(true), Climber));
        } else {
            CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.ShootArm(false), Climber));
        }

        if (OPERATOR.getRightY() > .5) {
            CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.NegativeShootArm(true), Climber));
        }*/
    }
}


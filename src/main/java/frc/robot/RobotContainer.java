package frc.robot;


import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Elevator.Positions;
import frc.robot.Robot.ControlMode;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.ColorAlignCommand;
import frc.robot.commands.StupidAlignCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.*;
import frc.robot.util.Gamepad;

import java.awt.*;
import java.util.Map;

import static frc.robot.Constants.Swerve.SPEED_MULTIPLIER;
import static frc.robot.Robot.AutoModeChooser;
import static frc.robot.Robot.ControlModeChooser;

public class RobotContainer {
    public final SwerveSubsystem Swerve = new SwerveSubsystem();
    public final LEDSubsystem LED = new LEDSubsystem();
    public final TongueSubsystem Tongue = new TongueSubsystem();
    public final ElevatorSubsystem Elevator = new ElevatorSubsystem(Tongue);
    public final ClimberSubsystem Climber = new ClimberSubsystem();
    public final ShooterSubsystem Shooter = new ShooterSubsystem();
    public final PoseEstimationSubsystem PoseEstimation = new PoseEstimationSubsystem(Swerve::getYaw, Swerve::getPositions);

    private final GenericEntry finalSpeedModifierEntry = Shuffleboard.getTab("config").add("final speed modifier", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

    Gamepad DRIVER = new Gamepad(Controller.DRIVER_PORT);
    Gamepad OPERATOR = new Gamepad(Controller.OPERATOR_PORT);
    private final JoystickButton slowSpeed = new JoystickButton(DRIVER, 4);
    private final JoystickButton highSpeed = new JoystickButton(DRIVER,3);

    private final StupidAlignCommand StupidAlignRight = new StupidAlignCommand(false, Swerve, LED);
    private final StupidAlignCommand StupidAlignLeft = new StupidAlignCommand(true, Swerve, LED);
    private final ColorAlignCommand ColorAlignRight = new ColorAlignCommand(false, Swerve, Tongue, LED);
    private final ColorAlignCommand ColorAlignLeft = new ColorAlignCommand(true, Swerve, Tongue, LED);

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
        Shuffleboard.getTab("main").add("zero elevator", new RunCommand(Elevator::zeroEncoders, Elevator)).withWidget(BuiltInWidgets.kCommand);
 //TURNING OFF THE AUTOBUILDER FOR NOW
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
        PathfindingCommand.warmupCommand().schedule();


        NamedCommands.registerCommand("Auto_Elevator", new InstantCommand(() -> Elevator.setPosition(Positions.Auto), Elevator));
        NamedCommands.registerCommand("L1_Elevator", new InstantCommand(() -> Elevator.setPosition(Positions.L1), Elevator));
        NamedCommands.registerCommand("L2_Elevator", new InstantCommand(() -> Elevator.setPosition(Positions.L2), Elevator));
        NamedCommands.registerCommand("L3_Elevator", new InstantCommand(() -> Elevator.setPosition(Positions.L3), Elevator));
        NamedCommands.registerCommand("Tongue_Extend", new InstantCommand(Tongue::extend, Tongue));
        NamedCommands.registerCommand("Tongue_Score", new InstantCommand(Tongue::setPosScore, Tongue));
        NamedCommands.registerCommand("Tongue_Carry", new InstantCommand(Tongue::setPosCarrying, Tongue));
        NamedCommands.registerCommand("Tongue_Receive", new InstantCommand(Tongue::setPosReceive, Tongue));
        NamedCommands.registerCommand("Intake_Elevator", new InstantCommand(() -> Elevator.setPosition(Positions.INTAKE), Elevator));
        NamedCommands.registerCommand("zeroGyro", new InstantCommand(Swerve::zeroGyro, Swerve));
        NamedCommands.registerCommand("Tongue_Auto", new InstantCommand(Tongue::setPosAuto, Tongue));
        NamedCommands.registerCommand("Shooter_Speaker", new InstantCommand(() -> Shooter.flywheelSpeaker(true, 1.0), Shooter));
        NamedCommands.registerCommand("Shooter_Off", new InstantCommand(() -> Shooter.flywheelSpeaker(false, 0.0), Shooter));


        new EventTrigger("shooting").whileTrue(Commands.print("shooting ball"));
        new EventTrigger("not shooting").whileTrue(Commands.print("not shooting ball"));

        new EventTrigger("climberReady").whileTrue(Commands.print("climber ready"));
        new EventTrigger("climberNotReady").whileTrue(Commands.print("climber not ready"));


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
                        slowSpeed,//slowMode,// DRIVER.getLeftStickButton(), // slow mode
                        highSpeed,//!slowMode,//DRIVER.getRightStickButton())); // fast mode
                        () ->finalSpeedModifierEntry.getDouble(1.0)));

        //Elevator.setDefaultCommand(
        //        new RunCommand(
        //                () -> Elevator.jogPositive(false),
        //                Elevator));
        //Tongue.setDefaultCommand(new RunCommand(()->Tongue.readSensor()),Tongue);
    }

    
    
    private void configureButtonBindings() {

       // new JoystickButton(DRIVER, 3)
        //        .onTrue(new RunCommand(Swerve::zeroGyro));

        new JoystickButton(DRIVER, 5)
                .onTrue(new RunCommand(Tongue::setPosL4, Tongue));
        new JoystickButton(DRIVER, 6)
                .onTrue(new RunCommand(Tongue::setPosL4, Tongue));
        new JoystickButton(DRIVER, 8).onTrue(new InstantCommand(Swerve::setSpeed));


        new JoystickButton(OPERATOR, 7)
                .whileTrue(new InstantCommand(() -> Elevator.jogPositive(true), Elevator))
                .onFalse(new InstantCommand(() -> Elevator.jogPositive(false), Elevator));

        new JoystickButton(OPERATOR, 8)
                .whileTrue(new InstantCommand(() -> Elevator.jogNegative(true), Elevator))
                .onFalse(new InstantCommand(() -> Elevator.jogNegative(false), Elevator));
        
        new JoystickButton(OPERATOR, 3)
        .whileTrue(new InstantCommand(() -> Shooter.flywheelAmp(true), Shooter));



   /*      new JoystickButton(OPERATOR, 7)
                .whileTrue(new RunCommand(() -> Elevator.jogPositive(true), Elevator));

        new JoystickButton(OPERATOR, 8)
                .whileTrue(new RunCommand(() -> Elevator.jogNegative(true), Elevator));*/

                

        new JoystickButton(OPERATOR, 3)
                .onTrue(new RunCommand(() -> Elevator.setPosition(Positions.ZERO), Elevator));
        new JoystickButton(OPERATOR, 4)
                .onTrue(new RunCommand(() -> Elevator.setPosition(Positions.INTAKE), Elevator));
        /*new JoystickButton(OPERATOR, 6)
                .onTrue(new RunCommand(() -> Elevator.setPosition(Positions.L3), Elevator));
        new JoystickButton(OPERATOR, 5)
                .onTrue(new RunCommand(() -> Elevator.setPosition(Positions.L4), Elevator));
        new JoystickButton(OPERATOR, 1)
                .whileTrue(new RunCommand(() -> Elevator.setPosition(Positions.INTAKE), Elevator));
        new JoystickButton(OPERATOR, 2)
                .whileTrue(new RunCommand(Tongue::setPosScore, Tongue));*/


        new JoystickButton(DRIVER, 1) // A left
            .whileTrue(new InstantCommand(() -> Shooter.flywheelAmp(true), Shooter))
            .onFalse(new InstantCommand(() -> Shooter.flywheelAmp(false), Shooter));
        new JoystickButton(DRIVER, 2) // B right
            .whileTrue(new InstantCommand(() -> Shooter.receive(true), Shooter))
            .onFalse(new InstantCommand(() -> Shooter.receive(false), Shooter));
      //  new JoystickButton(DRIVER, 1).
       //         whileTrue(Swerve.driveToPose());
//        new JoystickButton(DRIVER, 1).whileTrue(new AlignCommand(false, Swerve));
        //new JoystickButton(DRIVER, 1).whileTrue(StupidAlignLeft); // A is left
        //new JoystickButton(DRIVER, 2).whileTrue(StupidAlignRight); // B is right
        new JoystickButton(DRIVER, 3).whileTrue(ColorAlignLeft);
        new JoystickButton(DRIVER, 4).whileTrue(ColorAlignRight);
    }

    public Command getAutonomousCommand() {
        /*try{
        // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("TJAuto15");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        } */
       // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return new PathPlannerAuto("SQUARE");
    }

    public void checkAnalogs() {
        if (OPERATOR.getRightTriggerAxis() > .5) {
            CommandScheduler.getInstance().schedule(new RunCommand(Tongue::setPosReceive, Tongue));
           // CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));
        }

        if (OPERATOR.getLeftTriggerAxis() > .5) {
            CommandScheduler.getInstance().schedule(new RunCommand(Tongue::setPosCarrying, Tongue));
          //  CommandScheduler.getInstance().schedule(new InstantCommand(() -> System.out.println("Command scheduled!")));
        }

        if (OPERATOR.getRightY() < -.5) {
            CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.ShootArm(true), Climber));
        } else {

            CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.ShootArm(false), Climber));
        }

        if (OPERATOR.getRightY() > .5) {
            CommandScheduler.getInstance().schedule(new RunCommand(() -> Climber.NegativeShootArm(true), Climber));
        }
    }
}

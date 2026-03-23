 
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public class ShootCommand extends Command {
    private final ShooterSubsystem Shooter;
    //private final GenericEntry rpm = Shuffleboard.getTab("main").add("dumb rpm", 5600).getEntry();

    public ShootCommand(
            ShooterSubsystem Shooter
    ) {
        this.Shooter = Shooter;

        addRequirements(Shooter);
    }

    @Override
    public void initialize() {
        Shooter.prefireContinuous(5600/*rpm.getDouble(5600)*/);
    }
    @Override
    public void execute() {
        Shooter.shootContinuous(5600/*rpm.getDouble(5600)*/);
   }

    @Override
    public void end(boolean interrupted) {
        Shooter.shootWithPID(false, 5600/*rpm.getDouble(5600)*/);
    }
}
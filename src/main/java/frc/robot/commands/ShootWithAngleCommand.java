 
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootWithAngleCommand extends Command {
    private final SwerveSubsystem Swerve;
    private final ShooterSubsystem Shooter;
    private final PoseEstimationSubsystem Poser;
    private final ProfiledPIDController thetaPID = new ProfiledPIDController(0.4, 0, 0, new Constraints(Constants.Swerve.MAX_ANGULAR_VELOCITY, 0.5));
    private final GenericEntry maxRpm = Shuffleboard.getTab("main").add("WA max rpm", 5600.0).getEntry();
    private final GenericEntry maxRpmDistance = Shuffleboard.getTab("main").add("WA max rpm distance", 4.6).getEntry();
    private final GenericEntry rpmOffset = Shuffleboard.getTab("main").add("WA rpm offset", 0).getEntry();
    private double thetaGoal = 0.0;
    private double rpm = 0;

    public ShootWithAngleCommand(
            SwerveSubsystem Swerve,
            ShooterSubsystem Shooter,
            PoseEstimationSubsystem Poser
    ) {
        this.Swerve = Swerve;
        this.Shooter = Shooter;
        this.Poser = Poser;

        addRequirements(Swerve, Shooter);
    }

    @Override
    public void initialize() {
        double hubX = 4.625467;
        double hubY = 3.431286;
        if (Robot.ALLIANCE == DriverStation.Alliance.Red) {
            hubX = 11.915521;
        }
        double dx = hubX - Poser.getPose().getX();
        double dy = hubY - Poser.getPose().getY();
        thetaGoal = Math.toDegrees(Math.atan2(dy, dx));

        double distance = Math.sqrt((dx*dx) + (dy*dy));
        rpm = (maxRpm.getDouble(5600) * distance) / maxRpmDistance.getDouble(4.6);
        rpm += rpmOffset.getDouble(0.0);

        Shooter.prefireContinuous(rpm);
    }
    @Override
    public void execute() {
        thetaPID.setGoal(thetaGoal);
        double deltaTheta = Poser.getPose().getRotation().getDegrees() - thetaGoal;
        System.out.println("deltaTheta=" + deltaTheta);

        Shooter.shootContinuous(rpm);
        if (deltaTheta < 5) {
            Shooter.shootContinuous(rpm);
        } else {
            Swerve.drive(new Translation2d(0, 0), thetaPID.calculate(Poser.getPose().getRotation().getDegrees()), false, true);
        }
   }

    @Override
    public void end(boolean interrupted) {
        Shooter.shootWithPID(false, rpm);
    }
}
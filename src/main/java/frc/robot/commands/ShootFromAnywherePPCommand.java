
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Set;

public class ShootFromAnywherePPCommand extends SequentialCommandGroup {
    private final SwerveSubsystem Swerve;
    private final ShooterSubsystem Shooter;
    private final PoseEstimationSubsystem Poser;
    private final GenericEntry maxRpm = Shuffleboard.getTab("main").add("SFAPP max rpm", 5600.0).getEntry();
    private final GenericEntry maxRpmDistance = Shuffleboard.getTab("main").add("SFAPP max rpm distance", 4.6).getEntry();
    private final GenericEntry rpmOffset = Shuffleboard.getTab("main").add("SFAPP rpm offset", 0).getEntry();
    private double thetaGoal = 0.0;
    private double rpm = 0;

    private Command generatePathfindCommand() {
        double hubX = 4.625467;
        double hubY = 3.431286;
        if (Robot.ALLIANCE == DriverStation.Alliance.Red) {
            hubX = 11.915521;
        }
        double x = Poser.getPose().getX();
        double y = Poser.getPose().getY();
        double dx = hubX - x;
        double dy = hubY - y;
        thetaGoal = Math.toDegrees(Math.atan2(dy, dx));

        double distance = Math.sqrt((dx*dx) + (dy*dy));
        rpm = (maxRpm.getDouble(3850) * distance) / maxRpmDistance.getDouble(4.6);
        rpm += rpmOffset.getDouble(0.0);

        var constraints = new PathConstraints(1.5, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(new Pose2d(x, y, Rotation2d.fromRadians(thetaGoal)), constraints, 0.0);
    }

    public ShootFromAnywherePPCommand(
            SwerveSubsystem Swerve,
            ShooterSubsystem Shooter,
            PoseEstimationSubsystem Poser
    ) {
        this.Swerve = Swerve;
        this.Shooter = Shooter;
        this.Poser = Poser;

        addRequirements(Swerve, Shooter);

        Shooter.prefireContinuous(rpm);

        addCommands(new DeferredCommand(this::generatePathfindCommand, Set.of(Swerve)));
        addCommands(new Shoot(Shooter, rpm));
    }

    private class Shoot extends Command {
        private final ShooterSubsystem Shooter;
        private double rpma = 0.0;

        public Shoot(
                ShooterSubsystem Shooter,
                double rpmb
        ) {
            this.Shooter = Shooter;
            this.rpma = rpmb;

            addRequirements(Shooter);
        }

        @Override
        public void execute() {
            Shooter.shootContinuous(rpma);
        }

        @Override
        public void end(boolean interrupted) {
            Shooter.shootWithPID(false, rpma);
        }
    }
}
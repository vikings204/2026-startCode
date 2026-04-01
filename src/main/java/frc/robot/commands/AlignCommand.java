package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignCommand extends SequentialCommandGroup {
    public AlignCommand(SwerveSubsystem Swerve, PoseEstimationSubsystem Poser, boolean isLeft) {
        addRequirements(Swerve);

        var target = findTargetPose(Poser, isLeft);
        var constraints = new PathConstraints(1.5, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
        var cmd = AutoBuilder.pathfindToPose(target, constraints, 0.0);

        addCommands(cmd);
    }

    private Pose2d findTargetPose(PoseEstimationSubsystem Poser, boolean isLeft) {
        //double y = Poser.getPose().getY();
        //double halfway = 4.035;
        if (Robot.ALLIANCE == DriverStation.Alliance.Blue) {
            if (/*y <= halfway*/!isLeft) {
                // right side
                return new Pose2d(2.185, 1.477, new Rotation2d(Units.degreesToRadians(40.0)));
            } else {
                // left side
                return new Pose2d(1.771, 6.49, new Rotation2d(Units.degreesToRadians(-40.0)));
            }
        } else if (Robot.ALLIANCE == DriverStation.Alliance.Red) {
            if (/*y <= halfway*/isLeft) {
                // left side
                return new Pose2d(14.77, 1.579, new Rotation2d(Units.degreesToRadians(140.0)));
            } else {
                // right side
                return new Pose2d(14.356, 6.592, new Rotation2d(-140.0));
            }
        }

        return null;
    }
}

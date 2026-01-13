 
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Map;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static frc.robot.Constants.Vision.*;

public class AlignCommand extends Command {
    private final SwerveSubsystem Swerve;
    private double initialTimestamp;

    private final ProfiledPIDController xPID = new ProfiledPIDController(0.3, 0, 0, new Constraints(Constants.Swerve.MAX_SPEED/8, 0.5));
    private final ProfiledPIDController yPID = new ProfiledPIDController(0.3, 0, 0, new Constraints(Constants.Swerve.MAX_SPEED/8, 0.5));
    private final ProfiledPIDController thetaPID = new ProfiledPIDController(0.3, 0, 0, new Constraints(Constants.Swerve.MAX_ANGULAR_VELOCITY/8, 0.5));
    private final GenericEntry secondsToShootEntry = Shuffleboard.getTab("config").add("seconds to shoot", (double) 2).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 5)).getEntry();


    private final GenericEntry xGoal = Shuffleboard.getTab("main").add("x goal", (double) 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    private final GenericEntry yGoal = Shuffleboard.getTab("main").add("y goal", (double) 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    private final GenericEntry thetaGoal = Shuffleboard.getTab("main").add("theta goal", (double) 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    private final GenericEntry tolerance = Shuffleboard.getTab("main").add("tolerance", 0.2).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

    private DoubleSubscriber txSub;
    private DoubleSubscriber tySub;
    private DoubleSubscriber tzSub;
    private DoubleSubscriber idSub;
    private DoubleSubscriber yawSub;
    private DoubleSubscriber pitchSub;
    private DoubleSubscriber rollSub;


    public AlignCommand(
            boolean isLeft,
            SwerveSubsystem Swerve
    ) {
        this.Swerve = Swerve;

        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        initialTimestamp = getFPGATimestamp();

        var ntInstance = NetworkTableInstance.getDefault();
        var ntTable = ntInstance.getTable("datatable");
        txSub = ntTable.getDoubleTopic("tx").subscribe(0);
        tySub = ntTable.getDoubleTopic("ty").subscribe(0);
        tzSub = ntTable.getDoubleTopic("tz").subscribe(0);
        idSub = ntTable.getDoubleTopic("id").subscribe(0);
        yawSub = ntTable.getDoubleTopic("yaw").subscribe(0);
        pitchSub = ntTable.getDoubleTopic("pitch").subscribe(0);
        rollSub = ntTable.getDoubleTopic("roll").subscribe(0);
    }
    @Override
    public void execute() {
        xPID.setGoal(xGoal.getDouble(0));
        yPID.setGoal(yGoal.getDouble(0));
        thetaPID.setGoal(thetaGoal.getDouble(0));

        System.out.println("id: " + idSub.get());
        double diffX = txSub.get() - xGoal.getDouble(0);
        double diffY = tySub.get() - yGoal.getDouble(0);
        double diffTheta = yawSub.get() - thetaGoal.getDouble(0);
        System.out.println("diffX=" + diffX + " diffY=" + diffY + " diffTheta=" + diffTheta);

        if (/*getFPGATimestamp() > initialTimestamp+secondsToShootEntry.getDouble(2) && */diffX < 0.1 && diffY < 0.1 && diffTheta < 5) { // need to also check if close enough
            // shoot?
            System.out.println("close enough OR no data");
        } else {
            Swerve.drive(new Translation2d(xPID.calculate(txSub.get()), yPID.calculate(tySub.get())), thetaPID.calculate(yawSub.get()), false, true); // isOpenLoop differs from teleop
        }

        //Swerve.drive(new Translation2d(-1*xPID.calculate(robotPose.getX()), yPID.calculate(robotPose.getY())), -1*thetaPID.calculate(Swerve.getYaw().getRadians()), false, true); // isOpenLoop differs from teleop
   }
    @Override
    public void end(boolean interrupted) {
    }
}
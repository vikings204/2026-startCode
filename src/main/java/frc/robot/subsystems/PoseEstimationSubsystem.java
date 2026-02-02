package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.Supplier;

import static frc.robot.Constants.Vision.*;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;


public class PoseEstimationSubsystem extends SubsystemBase {
    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private double[] arrayForDashboard = new double[]{0, 0, 0};
    private double previousTimestamp = 0.0;

    private final DoubleSubscriber pxSub;
    private final DoubleSubscriber pySub;
    private final DoubleSubscriber tsSub;
    private final DoubleSubscriber delaySub;
    private final DoubleSubscriber tagsSub;

    public PoseEstimationSubsystem(
            Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier) {

        this.rotationSupplier = rotationSupplier;
        this.modulePositionSupplier = modulePositionSupplier;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.SWERVE_KINEMATICS,
                rotationSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(),
                STATE_STANDARD_DEVIATIONS,
                VISION_STANDARD_DEVIATIONS
                );

        var ntInstance = NetworkTableInstance.getDefault();
        var ntTable = ntInstance.getTable("datatable");
        pxSub = ntTable.getDoubleTopic("px").subscribe(0.0);
        pySub = ntTable.getDoubleTopic("py").subscribe(0.0);
        tsSub = ntTable.getDoubleTopic("ts").subscribe(0.0);
        delaySub = ntTable.getDoubleTopic("delay").subscribe(0.0);
        tagsSub = ntTable.getDoubleTopic("tags").subscribe(0.0);

        Shuffleboard.getTab("field").add("pose est field", field).withWidget(BuiltInWidgets.kField).withSize(8, 5);
        //Shuffleboard.getTab("main").addNumber("pose X", poseEstimator.getEstimatedPosition()::getX);
        //Shuffleboard.getTab("main").addNumber("pose Y", poseEstimator.getEstimatedPosition()::getY);
        //Shuffleboard.getTab("main").addNumber("gyro angle", poseEstimator.getEstimatedPosition().getRotation()::getDegrees);
        Shuffleboard.getTab("main").addNumber("pose X", () -> arrayForDashboard[0]);
        Shuffleboard.getTab("main").addNumber("pose Y", () -> arrayForDashboard[1]);
        Shuffleboard.getTab("main").addNumber("pose theta", () -> arrayForDashboard[2]);
        Shuffleboard.getTab("field").addNumber("pose X", () -> arrayForDashboard[0]);
        Shuffleboard.getTab("field").addNumber("pose Y", () -> arrayForDashboard[1]);
        Shuffleboard.getTab("field").addNumber("pose theta", () -> arrayForDashboard[2]);
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.updateWithTime(getFPGATimestamp(), rotationSupplier.get(), modulePositionSupplier.get());

        if (VISION_ENABLED) {
            double ts = tsSub.get();
            if (ts != previousTimestamp) {
                previousTimestamp = ts;
                Pose2d p = new Pose2d(pxSub.get(), pySub.get(), poseEstimator.getEstimatedPosition().getRotation());
                // map to robot
                poseEstimator.addVisionMeasurement(p, getFPGATimestamp()-delaySub.get());
            }
        }

        // Set the pose on the dashboard
        var dashboardPose = poseEstimator.getEstimatedPosition();
        field.setRobotPose(dashboardPose);
        arrayForDashboard = new double[]{dashboardPose.getX(), dashboardPose.getY(), dashboardPose.getRotation().getDegrees()};
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

}
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.ArrayDeque;
import java.util.function.Supplier;

import static frc.robot.Constants.Vision.*;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;


public class PoseEstimationSubsystem extends SubsystemBase {
    private final Supplier<Rotation2d> yawSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private double[] arrayForDashboard = new double[]{0, 0, 0};
    private int numTagsVisible = 0;

    private final LimelightSucks camera1;
    private final LimelightSucks camera2;
    private final YawHistory yawHistory;

    public PoseEstimationSubsystem(
            Supplier<Rotation2d> yawSupplier,
            Supplier<SwerveModulePosition[]> modulePositionSupplier,
            Supplier<ChassisSpeeds> speedsSupplier) {

        this.yawSupplier = yawSupplier;
        this.modulePositionSupplier = modulePositionSupplier;
        this.speedsSupplier = speedsSupplier;

        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.SWERVE_KINEMATICS,
                yawSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(),
                STATE_STANDARD_DEVIATIONS,
                STATE_STANDARD_DEVIATIONS
        );

        camera1 = new LimelightSucks("camera1");
        camera2 = new LimelightSucks("camera2");

        yawHistory = new YawHistory();

        Shuffleboard.getTab("main").add("pose est field", field).withWidget(BuiltInWidgets.kField).withSize(8, 5);
        //Shuffleboard.getTab("main").addNumber("pose X", poseEstimator.getEstimatedPosition()::getX);
        //Shuffleboard.getTab("main").addNumber("pose Y", poseEstimator.getEstimatedPosition()::getY);
        //Shuffleboard.getTab("main").addNumber("gyro angle", poseEstimator.getEstimatedPosition().getRotation()::getDegrees);
        Shuffleboard.getTab("main").addNumber("pose X", () -> arrayForDashboard[0]);
        Shuffleboard.getTab("main").addNumber("pose Y", () -> arrayForDashboard[1]);
        Shuffleboard.getTab("main").addNumber("pose theta", () -> arrayForDashboard[2]);
        Shuffleboard.getTab("main").addNumber("num tags", () -> numTagsVisible);
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        double now = getFPGATimestamp();
        poseEstimator.updateWithTime(now, yawSupplier.get(), modulePositionSupplier.get());
        yawHistory.update(yawSupplier.get().getDegrees(), now);

        if (VISION_ENABLED) {
            camera1.periodic();
            camera2.periodic();
            var cam1 = camera1.getMeasurement();
            var cam2 = camera2.getMeasurement();
            if (cam1 != null || cam2 != null) {
                var measurement = cam1 != null ? cam1 : cam2;
                var p = (Pose2d) measurement[0];
                var yaw = (double) measurement[1];
                var delay = (double) measurement[2];
                var numTags = (double) measurement[3];

                // adjust std devs by robot speeds
                /*
                ChassisSpeeds speeds = speedsSupplier.get();
                double avgSpeed = (speeds.vxMetersPerSecond + speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond) / 3;
                if (avgSpeed < MIN_SPEED_FOR_STD_DEV) {
                    if (getFPGATimestamp() < 90) {
                        // robot probably just started up and is sitting on the field so trust vision
                        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION));
                    } else {
                        // robot is still, basically ignore vision
                        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(IGNORE_VISION_STANDARD_DEVIATION, IGNORE_VISION_STANDARD_DEVIATION, IGNORE_VISION_STANDARD_DEVIATION));
                    }
                } else if (avgSpeed > MAX_SPEED_FOR_STD_DEV) {
                    // robot is moving fast, trust vision (idk if this is reasonable)
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION));
                } else {
                    // saihaj function
                    // https://www.desmos.com/calculator/rvbx3meynf
                    double stdDev = ((avgSpeed - MIN_SPEED_FOR_STD_DEV) / (MAX_SPEED_FOR_STD_DEV - MIN_SPEED_FOR_STD_DEV)) * (TRUST_VISION_STANDARD_DEVIATION-IGNORE_VISION_STANDARD_DEVIATION) + IGNORE_VISION_STANDARD_DEVIATION;
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, stdDev));
                }
                */

                // adjust std devs by yaw difference
                double yawDiff = Math.abs(yawHistory.getYawAtTime(now-delay) - yaw);
                if (yawDiff < 1.5) {
                    // could adjust the above number and interpolate, etc.
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION, TRUST_VISION_STANDARD_DEVIATION));
                } else {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(IGNORE_VISION_STANDARD_DEVIATION, IGNORE_VISION_STANDARD_DEVIATION, IGNORE_VISION_STANDARD_DEVIATION));
                }

                poseEstimator.addVisionMeasurement(p, now-delay);
                numTagsVisible = (int) numTags;
            } else {
                numTagsVisible = 0;
            }
        }

//        double hubX = 4.625467;
//        double hubY = 3.431286;
//        if (Robot.ALLIANCE == DriverStation.Alliance.Red) {
//            hubX = 11.915521;
//        }
//        double dx = hubX - getPose().getX();
//        double dy = hubY - getPose().getY();
//        System.out.println(Math.toDegrees(Math.atan2(dy, dx)));

        // Set the pose on the dashboard
        var dashboardPose = poseEstimator.getEstimatedPosition();
        field.setRobotPose(dashboardPose);
        arrayForDashboard = new double[]{Math.round(dashboardPose.getX()*1000.0)/1000.0, Math.round(dashboardPose.getY()*1000.0)/1000.0, Math.round(dashboardPose.getRotation().getDegrees()*1000.0)/1000.0};
       // System.out.println(arrayForDashboard[2]+" "+ poseEstimator.getEstimatedPosition().getRotation());

    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setPose(Pose2d newPose) {
        poseEstimator.resetPosition(yawSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetPose() {
        setPose(new Pose2d());
    }

    private class LimelightSucks {
        private double previousTimestamp = 0.0;
        private final DoubleSubscriber pxSub;
        private final DoubleSubscriber pySub;
        private final DoubleSubscriber yawSub;
        private final DoubleSubscriber tsSub;
        private final DoubleSubscriber delaySub;
        private final DoubleSubscriber tagsSub;
        private Pose2d p;
        private double yaw;
        private double delay;
        private double numTags;

        public LimelightSucks(String network_name) {
            var ntInstance = NetworkTableInstance.getDefault();
            var ntTable = ntInstance.getTable(network_name);
            pxSub = ntTable.getDoubleTopic("px").subscribe(0.0);
            pySub = ntTable.getDoubleTopic("py").subscribe(0.0);
            yawSub = ntTable.getDoubleTopic("yaw").subscribe(0.0);
            tsSub = ntTable.getDoubleTopic("timestamp").subscribe(0.0);
            delaySub = ntTable.getDoubleTopic("delay").subscribe(0.0);
            tagsSub = ntTable.getDoubleTopic("tags").subscribe(0.0);
        }

        public void periodic() {
            double ts = tsSub.get();
            if (ts != previousTimestamp) {
                previousTimestamp = ts;
                p = new Pose2d(Math.sqrt((pxSub.get() * pxSub.get())), Math.sqrt((pySub.get() * pySub.get())), poseEstimator.getEstimatedPosition().getRotation());
                p = p.transformBy(new Transform2d()); // cam to robot center (already done with multicam)
                yaw = yawSub.get();
                delay = delaySub.get();
                numTags = tagsSub.get();
            }
        }

        public Object[] getMeasurement() {
            if (p != null) {
                Object[] arr = {p, yaw, delay, numTags};
                p = null;
                yaw = 0.0;
                delay = 0.0;
                numTags = 0.0;
                return arr;
            } else {
                return null;
            }
        }
    }

    private class YawHistory {

        private static final double MAX_AGE_SECONDS = 3.0;

        private record Sample(double time, double yaw) {}

        private final ArrayDeque<Sample> history = new ArrayDeque<>(200);

        public void update(double yawDegrees, double time) {
            history.addLast(new Sample(time, yawDegrees));

            double cutoff = time - MAX_AGE_SECONDS;
            while (!history.isEmpty() && history.peekFirst().time() < cutoff)
                history.pollFirst();
        }

        public double getYawAtTime(double timestamp) {
            if (history.isEmpty()) return 0.0;
            if (timestamp <= history.peekFirst().time()) return history.peekFirst().yaw();
            if (timestamp >= history.peekLast().time())  return history.peekLast().yaw();

            Sample prev = null;
            for (Sample s : history) {
                if (s.time() > timestamp) {
                    double alpha = (timestamp - prev.time()) / (s.time() - prev.time());
                    return prev.yaw() + alpha * (s.yaw() - prev.yaw());
                }
                prev = s;
            }
            return history.peekLast().yaw();
        }
    }
}
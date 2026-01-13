package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.Map;

import static frc.robot.Constants.Swerve.*;


public class SwerveSubsystem extends SubsystemBase {
    public Pigeon2 gyro = new Pigeon2(PIGEON2_ID, "rio");
    public final SwerveModule[] modules; // Array of the 4 swerve modules
//    private final SwerveDrivePoseEstimator TESTPOSER = new SwerveDrivePoseEstimator(
//            Constants.Swerve.SWERVE_KINEMATICS,
//            new Rotation2d(),
//            new SwerveModulePosition[]{
//                    new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
//            },
//            new Pose2d()
//    );

    public SwerveSubsystem() {
        var toApply = new Pigeon2Configuration();
        gyro.getConfigurator().apply(toApply);

        zeroGyro();
        modules = new SwerveModule[]{
                        new SwerveModule(0, Mod0.DRIVE_MOTOR_ID, Mod0.ANGLE_MOTOR_ID, Mod0.ANGLE_OFFSET, Mod0.CAN_CODER_ID), //Each Constant set is specific to a motor pair
                        new SwerveModule(1, Mod1.DRIVE_MOTOR_ID, Mod1.ANGLE_MOTOR_ID, Mod1.ANGLE_OFFSET, Mod1.CAN_CODER_ID),
                        new SwerveModule(2, Mod2.DRIVE_MOTOR_ID, Mod2.ANGLE_MOTOR_ID, Mod2.ANGLE_OFFSET, Mod2.CAN_CODER_ID),
                        new SwerveModule(3, Mod3.DRIVE_MOTOR_ID, Mod3.ANGLE_MOTOR_ID, Mod3.ANGLE_OFFSET, Mod3.CAN_CODER_ID)
                };

       
        for (SwerveModule mod : modules) {
            Shuffleboard.getTab("swerve").addNumber("position: module " + mod.moduleNumber, () -> mod.getPosition().distanceMeters);
            Shuffleboard.getTab("swerve").addNumber("angle: module " + mod.moduleNumber, mod.getPosition().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180));
            Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle::getDegrees).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -180, "max", 180));
            Shuffleboard.getTab("swerve").addNumber("Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond);
        }
        Shuffleboard.getTab("main").addNumber("gyro angle", () -> {
            Rotation2d yaw = getYaw();
            return yaw.getDegrees();
        });
    }

    public void setSpeed(){
        if (Constants.Swerve.SPEED_MULTIPLIER <.5){
            Constants.Swerve.SPEED_MULTIPLIER = .85;
        }
        else{
            Constants.Swerve.SPEED_MULTIPLIER = .125;
        }
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                SWERVE_KINEMATICS.toSwerveModuleStates(fieldRelative ?
                        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()) :
                        new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        //System.out.println("Current Heading: "+getYaw());
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : modules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
        }
    }

    public ChassisSpeeds getSpeeds() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = SWERVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : modules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : modules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }
    public void setX(){
        for (SwerveModule mod : modules) {
            mod.resetToAbsolute();
        }
    }

    public void setGyro(double yaw) {
        gyro.setYaw(yaw);
    }

    public Rotation2d getYaw() {
        return (GYRO_INVERT)
                ? Rotation2d.fromDegrees(360 - gyro.getAngle())
                : Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetEncoders(){
        for (int i = 0; i<4; i++){
            modules[i].resetToAbsolute();
          }
    }

    public void zeroDriveEncoders() {
        for (SwerveModule mod : modules) {
            mod.zeroDriveEncoder();
        }
    }

    @Override
    public void periodic() {
        //TESTPOSER.update(new Rotation2d(), getPositions());
        //System.out.println("x=" + TESTPOSER.getEstimatedPosition().getX()*(4/2.8) + "  y=" + TESTPOSER.getEstimatedPosition().getY()*(4/2.8));
    }
    public Command driveToPose(){//Pose2d pose) {
    Translation2d t = new Translation2d(1.0,0.0);
    Pose2d test = new Pose2d(t, new Rotation2d(0.0));
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        1.0, 1.0,
        Units.degreesToRadians(360), Units.degreesToRadians(360));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        test,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }


}

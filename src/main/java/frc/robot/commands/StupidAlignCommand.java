package frc.robot.commands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public class StupidAlignCommand extends Command {
    private final SwerveSubsystem Swerve;
    private final LEDSubsystem LED;
    private final boolean isLeft;

    private IntegerSubscriber idSub;
    private DoubleSubscriber txSub;
//    private DoubleSubscriber tySub;
    private DoubleSubscriber tzSub;
//    private DoubleSubscriber yawSub;
//    private DoubleSubscriber pitchSub;
//    private DoubleSubscriber rollSub;
    private boolean zeroedWheels = false;
    private boolean doneWithTag = false;

    private final SwerveDrivePoseEstimator poser = new SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_KINEMATICS,
            new Rotation2d(),
            new SwerveModulePosition[]{
                    new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
            },
            new Pose2d()/*,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(1.5, 1.5, 1.5)*/
    );


    public StupidAlignCommand(
            boolean isLeft,
            SwerveSubsystem Swerve,
            LEDSubsystem LED
    ) {
        this.isLeft = isLeft;
        this.Swerve = Swerve;
        this.LED = LED;

        var ntInstance = NetworkTableInstance.getDefault();
        var ntTable = ntInstance.getTable("datatable");
        idSub = ntTable.getIntegerTopic("id").subscribe(0);
        txSub = ntTable.getDoubleTopic("tx").subscribe(0);
//        tySub = ntTable.getDoubleTopic("ty").subscribe(0);
        tzSub = ntTable.getDoubleTopic("tz").subscribe(0);
//        yawSub = ntTable.getDoubleTopic("yaw").subscribe(0);
//        pitchSub = ntTable.getDoubleTopic("pitch").subscribe(0);
//        rollSub = ntTable.getDoubleTopic("roll").subscribe(0);

        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        if (idSub.get() == 0) {
            System.out.println("align: no tag");
            LED.setPattern(LEDSubsystem.BlinkinPattern.DARK_RED);
            this.cancel();
        } else {
            System.out.println("align: found tag, aligning");
            LED.setPattern(LEDSubsystem.BlinkinPattern.SHOT_BLUE);
            zeroedWheels = false;
            doneWithTag = false;
            poser.resetPosition(new Rotation2d(), new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}, new Pose2d());
        }
    }

    @Override
    public void execute() {
        long id = idSub.get();

        if (id == 0) {
            doneWithTag = true;
        }

        double estimatedX = txSub.get();
        double estimatedY = tzSub.get();
        if (doneWithTag) {
            if (!zeroedWheels) {
                Swerve.zeroDriveEncoders();
                zeroedWheels = true;
            }

            // update with wheel odom
            poser.update(new Rotation2d(), Swerve.getPositions());

            estimatedX -= poser.getEstimatedPosition().getX()*(Constants.Swerve.WHEEL_DIAMETER_REAL/Constants.Swerve.WHEEL_DIAMETER);
            estimatedY -= poser.getEstimatedPosition().getY()*(Constants.Swerve.WHEEL_DIAMETER_REAL/Constants.Swerve.WHEEL_DIAMETER);
        }

        if (id == 0) {
            System.out.println("A x=" + poser.getEstimatedPosition().getX()*(Constants.Swerve.WHEEL_DIAMETER_REAL/Constants.Swerve.WHEEL_DIAMETER) + " y=" + poser.getEstimatedPosition().getY()*(Constants.Swerve.WHEEL_DIAMETER_REAL/Constants.Swerve.WHEEL_DIAMETER));
        } else {
            System.out.println("B x=" + estimatedX + " y=" + estimatedY);
        }

        double diffX = estimatedX - (isLeft ? 1 : -1) * 0.172;
        double diffY = estimatedY - .65;

        double speed = 0.3;
        double goX = diffX > 0 ? speed : -speed;
        double goY = diffY > 0 ? speed : -speed;

        if (Math.abs(diffX) < 0.01) {
            goX = 0;
        }
        if (Math.abs(diffY) < 0.01) {
            goY = 0;   
        }

        if (goX == 0 && goY == 0) {
            System.out.println("align: close enough");
            LED.setPattern(LEDSubsystem.BlinkinPattern.LIME);
            this.cancel();
        } else {
            Swerve.drive(new Translation2d(goX, goY), 0, false, true);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        poser.resetPosition(new Rotation2d(), new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()}, new Pose2d());
    }
}
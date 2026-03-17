package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;

import static frc.robot.Constants.Shooter.*;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax mainMotor;
    private final SparkClosedLoopController mainController;
    private final SparkMaxConfig mainMotorConfig;
    private final SparkMax kickMotor;
    private final SparkClosedLoopController kickController;
    private final SparkMaxConfig kickMotorConfig;
    private final SparkMax vectorMotor;
    private final SparkMaxConfig vectorMotorConfig;

    public ShooterSubsystem() {
        //led = ledSubsystem;
        // SET SHHOOTERMOTOR ID FROM CONSTANTS WHEN SET
        mainMotor = new SparkMax(MAIN_MOTOR_ID, MotorType.kBrushless);
        mainMotorConfig = new SparkMaxConfig();
        mainController = mainMotor.getClosedLoopController();
        kickMotor = new SparkMax(KICK_MOTOR_ID, MotorType.kBrushless);
        kickMotorConfig = new SparkMaxConfig();
        kickController = kickMotor.getClosedLoopController();
        vectorMotor = new SparkMax(VECTOR_MOTOR_ID, MotorType.kBrushless);
        vectorMotorConfig = new SparkMaxConfig();
        configMotors();

        // needed to do closed loop velocity control (optimal for high throughput)
        Shuffleboard.getTab("debug").addDouble("shooter main velocity", mainMotor.getEncoder()::getVelocity);
        Shuffleboard.getTab("debug").addDouble("shooter kick velocity", kickMotor.getEncoder()::getVelocity);
        Shuffleboard.getTab("debug").addDouble("shooter vector velocity", vectorMotor.getEncoder()::getVelocity);
    }

    private void configMotors() {
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(mainMotor, Usage.kVelocityOnly, mainMotorConfig);
        mainMotorConfig.smartCurrentLimit(40);
        mainMotorConfig.inverted(false);
        mainMotorConfig.idleMode(IdleMode.kBrake);
        mainMotorConfig.voltageCompensation(12.0);
        mainMotorConfig.encoder
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(8)
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(8);
        mainMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.3125, 0, 0.001)
                .outputRange(-1, 1);
        mainMotorConfig.closedLoop.feedForward
                .kV(0.2)
                .kA(0.4);
        mainMotor.configure(mainMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(kickMotor, Usage.kVelocityOnly, kickMotorConfig);
        kickMotorConfig.smartCurrentLimit(40);
        kickMotorConfig.inverted(true);
        kickMotorConfig.idleMode(IdleMode.kBrake);
        kickMotorConfig.voltageCompensation(12.0);
        kickMotorConfig.encoder
                .quadratureAverageDepth(2)
                .quadratureMeasurementPeriod(8)
                .uvwAverageDepth(2)
                .uvwMeasurementPeriod(8);
        kickMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.3125, 0, 0.001)
                .outputRange(-1, 1);
        kickMotorConfig.closedLoop.feedForward
                .kV(0.2)
                .kA(0.4);
        kickMotor.configure(kickMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(vectorMotor, Usage.kVelocityOnly, vectorMotorConfig);
        vectorMotorConfig.smartCurrentLimit(40);
        vectorMotorConfig.inverted(true);
        vectorMotorConfig.idleMode(IdleMode.kBrake);
        vectorMotorConfig.voltageCompensation(12.0);
        vectorMotor.configure(vectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void shootMotor(boolean shoot) {
        if (shoot) {
            mainMotor.set(1);
            kickMotor.set(1);
            vectorMotor.set(1);
        } else {
            mainMotor.set(0);
            kickMotor.set(0);
            vectorMotor.set(0);
        }
    }

    public void shootWithPID(boolean shoot) {
        if (shoot) {
            mainController.setSetpoint(5600, SparkBase.ControlType.kVelocity);
            kickController.setSetpoint(5600, SparkBase.ControlType.kVelocity);
            //vectorController.setSetpoint(1, SparkBase.ControlType.kVelocity);
            vectorMotor.set(1);
        } else {
            mainMotor.set(0);
            kickMotor.set(0);
            vectorMotor.set(0);
        }
    }

    @Override
    public void periodic() {}
}
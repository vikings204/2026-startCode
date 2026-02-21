package frc.robot.subsystems;

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
    private final SparkMaxConfig mainMotorConfig;
    private final SparkMax kickMotor;
    private final SparkMaxConfig kickMotorConfig;
    private final SparkMax vectorMotor;
    private final SparkMaxConfig vectorMotorConfig;

    public ShooterSubsystem() {
        //led = ledSubsystem;
        // SET SHHOOTERMOTOR ID FROM CONSTANTS WHEN SET
        mainMotor = new SparkMax(MAIN_MOTOR_ID, MotorType.kBrushless);
        mainMotorConfig = new SparkMaxConfig();
        kickMotor = new SparkMax(KICK_MOTOR_ID, MotorType.kBrushless);
        kickMotorConfig = new SparkMaxConfig();
        vectorMotor = new SparkMax(VECTOR_MOTOR_ID, MotorType.kBrushless);
        vectorMotorConfig = new SparkMaxConfig();
        configMotors();

        // needed to do closed loop velocity control (optimal for high throughput)
        Shuffleboard.getTab("debug").addDouble("shooter main velocity", mainMotor.getEncoder()::getVelocity);
        Shuffleboard.getTab("debug").addDouble("shooter kick velocity", kickMotor.getEncoder()::getVelocity);
        Shuffleboard.getTab("debug").addDouble("shooter vector velocity", vectorMotor.getEncoder()::getVelocity);
    }

    private void configMotors() {
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(mainMotor, Usage.kAll, mainMotorConfig);
        mainMotorConfig.smartCurrentLimit(40);
        mainMotorConfig.inverted(false);
        mainMotorConfig.idleMode(IdleMode.kBrake);
        mainMotorConfig.voltageCompensation(12.0);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(kickMotor, Usage.kAll, kickMotorConfig);
        kickMotorConfig.smartCurrentLimit(40);
        kickMotorConfig.inverted(false);
        kickMotorConfig.idleMode(IdleMode.kBrake);
        kickMotorConfig.voltageCompensation(12.0);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(vectorMotor, Usage.kAll, vectorMotorConfig);
        vectorMotorConfig.smartCurrentLimit(40);
        vectorMotorConfig.inverted(false);
        vectorMotorConfig.idleMode(IdleMode.kBrake);
        vectorMotorConfig.voltageCompensation(12.0);
    }

    public void shootMotor(boolean shoot, double speed) {
        if (shoot) {
            mainMotor.set(speed);
            kickMotor.set(1);
            vectorMotor.set(1);

        } else {
            mainMotor.set(0);
            kickMotor.set(0);
            vectorMotor.set(0);
        }
    }

    public void intake(boolean shoot, boolean reverse) {}

    @Override
    public void periodic() {}
}
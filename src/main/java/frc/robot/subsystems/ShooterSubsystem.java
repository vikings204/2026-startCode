package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor_1;
    private final SparkMaxConfig shooterMotor_1_Config;
    private final SparkMax shooterMotor_2;
    private final SparkMaxConfig shooterMotor_2_Config;
    private final SparkMax shooterMotor_3;
    private final SparkMaxConfig shooterMotor_3_Config;

    public ShooterSubsystem() {
        //led = ledSubsystem;
        // SET SHHOOTERMOTOR ID FROM CONSTANTS WHEN SET
        shooterMotor_1 = new SparkMax(51, MotorType.kBrushless);
        shooterMotor_1_Config = new SparkMaxConfig();
        shooterMotor_2 = new SparkMax(52, MotorType.kBrushless);
        shooterMotor_2_Config = new SparkMaxConfig();
        shooterMotor_3 = new SparkMax(53, MotorType.kBrushless);
        shooterMotor_3_Config = new SparkMaxConfig();
        configMotors();
    }

    private void configMotors() {
        //shooterMotor_1.restoreFactoryDefaults();
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(shooterMotor_1, Usage.kAll, shooterMotor_1_Config);
        shooterMotor_1_Config.smartCurrentLimit(40);
        shooterMotor_1_Config.inverted(false);
        shooterMotor_1_Config.idleMode(IdleMode.kBrake);
        shooterMotor_1_Config.voltageCompensation(12.0);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(shooterMotor_2, Usage.kAll, shooterMotor_2_Config);
        shooterMotor_2_Config.smartCurrentLimit(40);
        shooterMotor_2_Config.inverted(false);
        shooterMotor_2_Config.idleMode(IdleMode.kBrake);
        shooterMotor_2_Config.voltageCompensation(12.0);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(shooterMotor_3, Usage.kAll, shooterMotor_3_Config);
        shooterMotor_3_Config.smartCurrentLimit(40);
        shooterMotor_3_Config.inverted(false);
        shooterMotor_3_Config.idleMode(IdleMode.kBrake);
        shooterMotor_3_Config.voltageCompensation(12.0);
    }

    public void shootMotor(boolean shoot, double speed) {
        if (shoot) {
            shooterMotor_1.set(speed);
            shooterMotor_2.set(1);
            shooterMotor_3.set(1);

        } else {
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            shooterMotor_3.set(0);
        }
    }

    public void intake(boolean shoot, boolean reverse) {}

    @Override
    public void periodic() {}
}
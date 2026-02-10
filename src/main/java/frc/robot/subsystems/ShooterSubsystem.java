package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;
import com.revrobotics.spark.SparkClosedLoopController;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor_1;
    private final SparkMaxConfig shooterMotor_1_Config;
    private final SparkMax shooterMotor_2;
    private final SparkMaxConfig shooterMotor_2_Config;
    private final SparkMax shooterMotor_3;
    private final SparkMaxConfig shooterMotor_3_Config;
    private boolean flywheelState = false;
    private boolean ignoreSensor = false;
    private final SendableChooser<Boolean> ignoreSensorChooser = new SendableChooser<>();
   // private final LEDSubsystem led;
    private boolean noteDetected;
    public boolean detecting = true;
    private final SparkClosedLoopController shooterMotor1_CLoopController;
    private final SparkClosedLoopController shooterMotor2_CLoopController;
    private final SparkClosedLoopController shooterMotor3_CLoopController;
    
   // private final GenericEntry entry;

    public ShooterSubsystem() {
        //led = ledSubsystem;
        // SET SHHOOTERMOTOR ID FROM CONSTANTS WHEN SET
        shooterMotor_1 = new SparkMax(51, MotorType.kBrushless);
        shooterMotor_1_Config = new SparkMaxConfig();
        shooterMotor_2 = new SparkMax(52, MotorType.kBrushless);
        shooterMotor_2_Config = new SparkMaxConfig();
        shooterMotor_3 = new SparkMax(53, MotorType.kBrushless);
        shooterMotor_3_Config = new SparkMaxConfig();
        shooterMotor1_CLoopController = shooterMotor_1.getClosedLoopController();
        shooterMotor2_CLoopController = shooterMotor_2.getClosedLoopController();
        shooterMotor3_CLoopController = shooterMotor_3.getClosedLoopController();
        configMotors();


        ignoreSensorChooser.addOption("deactivate sensor", false);
        ignoreSensorChooser.setDefaultOption("sensor is active", true);
        Shuffleboard.getTab("config").add("ignore intake sensor", ignoreSensorChooser).withWidget(BuiltInWidgets.kSplitButtonChooser).withSize(2, 1);
        ignoreSensorChooser.onChange((Boolean val) -> ignoreSensor = val);

        //Shuffleboard.getTab("debug").addNumber("Intake Motor Current", intakeMotor::getOutputCurrent);
        //entry = Shuffleboard.getTab("config").add("amp speed").getEntry();
        /* 
        Shuffleboard.getTab("debug").addNumber("NOTE DISTANCE", () -> {
            noteDetected = prox > INTAKE_SENSOR_THRESHOLD;
            return prox;
        });
        */
        Shuffleboard.getTab("debug").addBoolean("DETECTED", () -> noteDetected);
        Shuffleboard.getTab("debug").addBoolean("detecting?", () -> detecting);
    }

    //configDriveMotor();
    private void configMotors() {
        //shooterMotor_1.restoreFactoryDefaults();
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(shooterMotor_1, Usage.kAll, shooterMotor_1_Config);
        shooterMotor_1_Config.smartCurrentLimit(40);
        shooterMotor_1_Config.inverted(false);
        shooterMotor_1_Config.idleMode(IdleMode.kBrake);
        shooterMotor_1_Config.voltageCompensation(12.0);
        shooterMotor1_CLoopController.setSetpoint(30, ControlType.kVelocity);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(shooterMotor_2, Usage.kAll, shooterMotor_2_Config);
        shooterMotor_2_Config.smartCurrentLimit(40);
        shooterMotor_2_Config.inverted(false);
        shooterMotor_2_Config.idleMode(IdleMode.kBrake);
        shooterMotor_2_Config.voltageCompensation(12.0);
        shooterMotor2_CLoopController.setSetpoint(60, ControlType.kVelocity);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(shooterMotor_3, Usage.kAll, shooterMotor_3_Config);
        shooterMotor_3_Config.smartCurrentLimit(40);
        shooterMotor_3_Config.inverted(false);
        shooterMotor_3_Config.idleMode(IdleMode.kBrake);
        shooterMotor_3_Config.voltageCompensation(12.0);
        shooterMotor3_CLoopController.setSetpoint(120, ControlType.kVelocity);
       
        
    }

    public void shootMotor(boolean shoot, double speed) {
        
        flywheelState = shoot;

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
/* 
    public void flywheelAmp(boolean shoot) {
        flywheelState = shoot;
        if (shoot) {
//             shooterMotor_1.set(AMP_SPEED);
            shooterMotor_1.set(1);
            shooterMotor_2.set(1);
            shooterMotor_3.set(1);
        } else {
            System.out.println("lol");
            shooterMotor_1.set(0);
            shooterMotor_2.set(0);
            shooterMotor_3.set(0);
        }
    }
*/
/* 
    public void receive(boolean shoot) {
        if (shoot && !noteDetected) {
            shooterMotor_1.set(-.5);
        } else {
            shooterMotor_1.set(0);
        }
    }
*/
    public void intake(boolean shoot, boolean reverse) {
        if (ignoreSensor) {
            if (shoot) {
                
            } else {
               
            }
        } else {
            var detected = noteDetected;
            if (!detecting) {
                detected = true;
            }
            if (flywheelState || reverse) {
                detected = false;
            }

            if (shoot && !detected) {
                
            } else {
                
            }
        }
    }

    @Override
    public void periodic() {
        if (noteDetected && !flywheelState) {
            detecting = false;
          
        } else if (flywheelState) {
         
        } else {
         
        }
    }
}
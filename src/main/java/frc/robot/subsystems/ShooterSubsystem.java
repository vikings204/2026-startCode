package frc.robot.subsystems;

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

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterMotor_1;
    private final SparkMaxConfig shooterMotor_1_Config;
    private boolean flywheelState = false;
    private boolean ignoreSensor = false;
    private final SendableChooser<Boolean> ignoreSensorChooser = new SendableChooser<>();
   // private final LEDSubsystem led;
    private boolean noteDetected;
    public boolean detecting = true;
   // private final GenericEntry entry;

    public ShooterSubsystem() {
        //led = ledSubsystem;
        // SET SHHOOTERMOTOR ID FROM CONSTANTS WHEN SET
        shooterMotor_1 = new SparkMax(43, MotorType.kBrushless);
        shooterMotor_1_Config = new SparkMaxConfig();
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
       
        
    }

    public void flywheelSpeaker(boolean shoot, double speed) {
        
        flywheelState = shoot;

        if (shoot) {
            shooterMotor_1.set(speed);

        } else {
            shooterMotor_1.set(0);
        }

    }

    public void flywheelAmp(boolean shoot) {
        flywheelState = shoot;
        if (shoot) {
//             shooterMotor_1.set(AMP_SPEED);
            shooterMotor_1.set(1);
        } else {
            System.out.println("lol");
            shooterMotor_1.set(0.000);
        }
    }

    public void receive(boolean shoot) {
        if (shoot && !noteDetected) {
            shooterMotor_1.set(-.5);
        } else {
            shooterMotor_1.set(0);
        }
    }

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
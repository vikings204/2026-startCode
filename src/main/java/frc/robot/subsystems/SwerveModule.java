package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

///import frc.robot.util.CANCoderUtil.CCU;y
//import frc.robot.util.CANCoderUtil.CCCUsage
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.util.ReduceCANUsage;
import frc.robot.util.ReduceCANUsage.Spark_Max.Usage;
import frc.robot.util.ReduceCANUsage.CANCoderUtil;
import frc.robot.util.ReduceCANUsage.CANCoderUtil.CCUsage;
import frc.robot.Robot;


import java.util.Map;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private final SparkMax driveMotor;
    private final SparkMax angleMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig angleConfig;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;
    public CANcoder angleEncoder;

    private final SparkClosedLoopController driveController;
    public final SparkClosedLoopController angleController;


    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    DRIVE_FF_S, DRIVE_FF_V, DRIVE_FF_A);

    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, Rotation2d angleOffset, int canCoderID) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        /* Angle Encoder Config */
         angleEncoder = new CANcoder(canCoderID);
        configAngleEncoder();

        angleMotor = new SparkMax(angleMotorID, MotorType.kBrushless);
        angleConfig = new SparkMaxConfig();
        integratedAngleEncoder = angleMotor.getEncoder();

        configAngleMotor();
        angleController = angleMotor.getClosedLoopController();


        /* Drive Motoar Config */
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveConfig = new SparkMaxConfig();
        configDriveMotor();
        driveController = driveMotor.getClosedLoopController();

        lastAngle = getState().angle;

        Shuffleboard.getTab("swervetest").addNumber("angleEncoderCurrent Reading " + moduleNumber, integratedAngleEncoder::getPosition).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 360));
        //REMOVED MOD.KABS AS I THINK IT WAS DEPRECATED
        Shuffleboard.getTab("swervetest").addNumber("angleMotorAbsEncoder Reading " + moduleNumber, angleMotor.getAnalog()::getVoltage);
    }
    
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not

        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void resetToAbsolute() {
        //angleMotor.getAnalog().setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        
        // 2025 UPDATED TO USE CANCODER AND SET POSITION ABS POSITION - NEED TO WORK ON TO FINALIZE
        //MIGHT HAVE TO CHANGE THIS BACK TO JUST GET THE ROBOT UP AND RUNNING!
        
        //double absolutePosition = (getCanCoder().getRotations() - angleOffset.getRotations())/ANGLE_GEAR_RATIO;
       // double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble()-angleOffset.getRotations();
       // System.out.println("Encoder" +moduleNumber+ "Absolute Position: "+absolutePosition);    

        
        
        System.out.println("Encoder "+moduleNumber+ " is set to Absolute Position");
        System.out.println("The offset is "+ angleOffset.getRotations());
        System.out.println("The Absolute Position is "+ angleEncoder.getAbsolutePosition().getValueAsDouble());
        double absolutePosition = angleEncoder.getAbsolutePosition().getValueAsDouble()-angleOffset.getRotations();
        if(absolutePosition<0){
            absolutePosition = 1+absolutePosition;
        }
        System.out.println("The Integrated encoder is reading: "+integratedAngleEncoder.getPosition());
        
        Timer.delay(.3);

        integratedAngleEncoder.setPosition(Math.abs(absolutePosition));
        Timer.delay(.3);

        System.out.println("Now the Integrated encoder is reading: "+integratedAngleEncoder.getPosition());


    }

        

    //JOE EDIT THIS

    private void configAngleEncoder() {
        //angleEncoder.getConfigurator().apply(angleEncoder.getConfigurator().defaultCANCoderConfig);
         CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
         angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
         
         //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);

        //angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    private void configAngleMotor() {
       
       //2025 This is Deprecated angleMotor.restoreFactoryDefaults();
        
       ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(angleMotor, Usage.kAll,angleConfig);
      
      /*  angleConfig.smartCurrentLimit(ANGLE_CURRENT_LIMIT) ;
       // replaced above angleMotor.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
       angleConfig.inverted(ANGLE_INVERT); 
       // replaced above angleMotor.setInverted(ANGLE_INVERT);
       angleConfig.idleMode(ANGLE_IDLE_MODE); 
       // replaced above angleMotor.setIdleMode(ANGLE_IDLE_MODE);
        angleConfig.encoder.positionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        angleConfig.closedLoop
            .feedbackƒabsoensor(FeedbackSensor.kPrimaryEncoder)
            //.pidf(.01, ANGLE_PID_I, ANGLE_PID_D, ANGLE_PID_FF)
            .pid(.01, ANGLE_PID_I, ANGLE_PID_D)
            .outputRange(-1,1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, ANGLE_POSITION_CONVERSION_FACTOR);
            */
            //angleMotor.configureI
            angleConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20)
                    .inverted(true);
                    //.setInverted(true);
            angleConfig.encoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering moton the MAXSwerve Module.
                    //.inverted(true)
                    .positionConversionFactor(1/ANGLE_GEAR_RATIO) // radians
                    .velocityConversionFactor(1); // radians per second
            angleConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(1, 0, 0)
                    .outputRange(-1,1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0,1)
                    .minOutput(-1)
                    .maxOutput(1);
            angleConfig.closedLoop.apply(angleConfig.closedLoop);
            angleConfig.apply(angleConfig);

        
            // angleController
        // Replaced above 
        //angleController.SetP(ANGLE_PID_P);
        //angleController.
        //angleController.setI(ANGLE_PID_I);
        //angleController.setD(ANGLE_PID_D);
        //angleController.setFF(ANGLE_PID_FF);
       // angleController.setFeedbackDevice(angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute));
        angleConfig.voltageCompensation(VOLTAGE_COMPENSATION);
       //REPLACED ABOVE angleMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
       angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
       //angleMotor.burnFlash();
        Timer.delay(.2);
      resetToAbsolute();
    }

    private void configDriveMotor() {

        //driveMotor.restoreFactoryDefaults();
        // ASK NEEV ReduceCANUsage.SparkMax.setSparkMaxBusUsage(driveMotor, Usage.kAll);
        ReduceCANUsage.Spark_Max.setCANSparkMaxBusUsage(driveMotor, Usage.kAll,driveConfig);

        driveConfig.smartCurrentLimit(DRIVE_CURRENT_LIMIT) ;
        // replaced above driveMotor.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
        driveConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        //driveMotor.setInverted(DRIVE_INVERT);
        // Both replaced above driveMotor.setIdleMode(DRIVE_IDLE_MODE);
        driveConfig.encoder
            .positionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
        // REPLACED ABOVE driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
        //driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR);
        driveConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .pidf(DRIVE_PID_P, DRIVE_PID_I, DRIVE_PID_D, DRIVE_PID_FF);

       /* REPLACE ABOVE
         driveController.setP(DRIVE_PID_P);
        driveController.setI(DRIVE_PID_I);
        driveController.setD(DRIVE_PID_D);
        driveController.setFF(DRIVE_PID_FF);
        */
        driveConfig.voltageCompensation(VOLTAGE_COMPENSATION);

        // replaced above driveMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //replaced above driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                     ClosedLoopSlot.kSlot0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }
    public void setDriveVoltage(Voltage desiredVoltage){
            driveMotor.setVoltage(desiredVoltage);
    }
    public void setAngleVoltage(Voltage desiredVoltage){
            angleMotor.setVoltage(desiredVoltage);
    }


    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle;
            /*SmartDashboard.putNumber("Angle Position Setting Mod" + moduleNumber, angle.getDegrees());
            SmartDashboard.putNumber("Encoder Position Setting without Offset" + moduleNumber, (((angle.getDegrees( ))/360)*1023));   
            SmartDashboard.putNumber("Encoder Position Setting with Offset" + moduleNumber, (((angle.getDegrees()+angleOffset.getDegrees())/360)*1023)); */
       // double targetVoltage = angle.getDegrees()- angleOffset.getDegrees();
       //angleMotor.set(.5);
        if (moduleNumber == 0){
            //System.out.println("Angle Position Setting Mod" + moduleNumber + ": " + angle.getRotations());
        }
        angleController.setReference(angle.getRotations(), ControlType.kPosition);
    
      
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        //SmartDashboard.putNumber("getAngleCall position Mod" + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees());
        //System.out.println("Encoder Position Mod "+moduleNumber+": "+(angleMotor.getSelectedSensorPosition()/1023)*360);
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
        return Rotation2d.fromRotations(integratedAngleEncoder.getPosition());
        //return Rotation2d.fromDegrees(getCanCoder().getDegrees() - angleOffset.getDegrees());
    }
    

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());

        // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        //return Rotation2d.fromDegrees(((angleMotor.getSelectedSensorPosition()/1023)*360)-angleOffset.getDegrees());
       //return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
       //return Rotation2d.fromDegrees(360.0*angleEncoder.getAbsolutePosition().getValueAsDouble());

       //return Rotation2d.fromDegrees((angleMotor.getAnalog(SparkAnalogSensor.Mode.kAbsolute).getVoltage()/3.3)*360);
    }
   
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    //public void resetEncoder() {
     //   integratedAngleEncoder.setPosition(0);
   // }

    public SwerveModulePosition getPosition() {
        //SmartDashboard.putNumber("Raw Angle Reading " + moduleNumber, (angleMotor.getSelectedSensorPosition()/1023)*360);
        //System.out.println("Encoder Position: "+((angleMotor.getSelectedSensorPosition()/1023)*360-angleOffset.getDegrees()));
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                //Rotation2d.fromDegrees(getCanCoder().getDegrees()- angleOffset.getDegrees())
                Rotation2d.fromRotations(integratedAngleEncoder.getPosition())
                
                /////////////
                //NEW CODE//
                ///////////
                
                //Rotation2d.fromDegrees(angleEncoder.getPosition().getValueAsDouble() - angleOffset.getDegrees())
        );

    }

    public void zeroDriveEncoder() {
        driveEncoder.setPosition(0.0);
    }
}

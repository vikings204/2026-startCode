package frc.robot.util;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.SensorInitializationStrategy;
//import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.Constants;


public final class CTREConfigs {
  //public CANCoderConfiguration swerveCanCoderConfig;
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
   // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
    //swerveCanCoderConfig.
    //swerveCanCoderConfig.initializationStrategy =
     //   SensorInitializationStrategy.BootToAbsolutePosition;
    //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }    
}

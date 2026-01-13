package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TongueSubsystem extends SubsystemBase {
    ServoHubConfig config = new ServoHubConfig();
    ServoHub hub = new ServoHub(3);
    ServoChannel pegServo;
    ServoChannel extenderServo;
    private final ColorSensorV3 sensor1;
    private boolean goodToShoot;


    public TongueSubsystem() {
        config.channel0
                .pulseRange(500, 1500, 2500)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
        config.channel1
                .pulseRange(500, 1500, 2500)
                .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
        hub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
        hub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 5000);

        pegServo = hub.getServoChannel(ChannelId.kChannelId0);
        pegServo.setPowered(true);
        pegServo.setEnabled(true);

        extenderServo = hub.getServoChannel(ChannelId.kChannelId1);
        extenderServo.setPowered(true);
        extenderServo.setEnabled(true); // moved from extend(), should work
        sensor1 = new ColorSensorV3(I2C.Port.kOnboard);
        //sensor1.configureProximitySensor(ColorSensorV3.ProximitySensorResolution.kProxRes11bit, ColorSensorV3.ProximitySensorMeasurementRate.kProxRate6ms);
        Shuffleboard.getTab("main").addBoolean("SHOOOOOOOT", () -> isCloseEnough()).withWidget(BuiltInWidgets.kBooleanBox);

    }

    public boolean isCloseEnough() {
        var rawColor = sensor1.getRawColor();
        SmartDashboard.putNumber("Red Raw", rawColor.red);
        SmartDashboard.putNumber("Green Raw", rawColor.green);
        SmartDashboard.putNumber("Blue Raw", rawColor.blue);

        int proximity = sensor1.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);
        return proximity > 100 && rawColor.red > 100;

    }

    public void setPosScore() {

        int pw = pegServo.getPulseWidth();
        //int pw = 1350;
        while (pw > 645) {
            pegServo.setPulseWidth(pw);
            pw -= .025;
            Timer.delay(.0015);
        }
    }

    public void setPosCarrying() {
        pegServo.setPulseWidth(1665);
    }

    public void setPosAuto() {
        pegServo.setPulseWidth(900);
    }

    public void setPosReceive() {
        pegServo.setPulseWidth(1050);
    }

    public void setPosL4() {
        pegServo.setPulseWidth(1505);
    }

    public void extend() {
        extenderServo.setPulseWidth(1008);
    }

    public void retract() {
        extenderServo.setPulseWidth(500);
    }

}

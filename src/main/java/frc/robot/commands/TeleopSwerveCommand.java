package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.Swerve.CONTROLLER_RAMP_DEGREE;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerveCommand extends Command {
    private final SwerveSubsystem s_Swerve;
    private final DoubleSupplier translationSup;    //+/- y direction
    private final DoubleSupplier strafeSup;         // +/- x direction
    private final DoubleSupplier rotationSup;       // spin CCW or CW
    private final BooleanSupplier robotCentricSup;  // always false

    private final SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);  //Limits the rate of change of the voltage output to the motor to some maximum value.  Would change if you wanted the robot to "ramp" faster
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);     // See above
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);   //Not really used with the brushed motors going to a position but could be.  We don't use the trapazoid pid profile suggested only because it hasn't been implemented yet

    private final GenericEntry finalSpeedModifierEntry;

    public TeleopSwerveCommand(
            SwerveSubsystem s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);  //Adds the swerve subsystem as a req of the command and will not schedule any other commands that require the swerve sub


        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        finalSpeedModifierEntry = Shuffleboard.getTab("config").add("final speed modifier", 1.0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }

    @Override
    public void execute() {
        double speedMultiplier = Constants.Swerve.SPEED_MULTIPLIER * finalSpeedModifierEntry.getDouble(1.0);

        //double translationVal = translationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(translationSup.getAsDouble(), CONTROLLER_RAMP_DEGREE));
        //double strafeVal = strafeLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(strafeSup.getAsDouble(),  CONTROLLER_RAMP_DEGREE));
        //double rotationVal = rotationLimiter.calculate(speedMultiplier * MathUtil.applyDeadband(rotationSup.getAsDouble(), CONTROLLER_RAMP_DEGREE));

        double translationVal = translationLimiter.calculate(speedMultiplier
                * MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controller.DEADBAND)
        );
        double strafeVal = strafeLimiter.calculate(speedMultiplier
                * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controller.DEADBAND)
        );
        double rotationVal = rotationLimiter.calculate(speedMultiplier
                * MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Controller.DEADBAND)
        );

        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}

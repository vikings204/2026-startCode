package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TongueSubsystem;

public class ColorAlignCommand extends Command {
    private final SwerveSubsystem Swerve;
    private final TongueSubsystem Tongue;
    private final LEDSubsystem LED;
    private final boolean isLeft;

    public ColorAlignCommand(
            boolean isLeft,
            SwerveSubsystem Swerve,
            TongueSubsystem Tongue,
            LEDSubsystem LED
    ) {
        this.isLeft = isLeft;
        this.Swerve = Swerve;
        this.Tongue = Tongue;
        this.LED = LED;

        addRequirements(Swerve);
    }

    @Override
    public void initialize() {
        LED.setPattern(LEDSubsystem.BlinkinPattern.SHOT_RED);
        System.out.println("color align: start");
    }

    @Override
    public void execute() {
        double speed = 0.3;
        double forwardSpeed = 0.15;
        Swerve.drive(new Translation2d(isLeft ? -speed : speed, forwardSpeed), 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return Tongue.isCloseEnough();
    }

    @Override
    public void end(boolean interrupted) {
        LED.setPattern(LEDSubsystem.BlinkinPattern.GREEN);
        System.out.println("color align: close enough");
    }
}
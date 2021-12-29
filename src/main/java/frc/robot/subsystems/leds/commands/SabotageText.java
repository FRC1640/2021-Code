package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.leds.LedSubsystem;

public class SabotageText extends CommandBase {

    LedSubsystem ledSubsystem;

    public SabotageText(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        for(int i = 0; i <= 299; i++) {
            ledSubsystem.setLed(i, 255, 140, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
    
}

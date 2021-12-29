package frc.robot.subsystems.leds.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.leds.LedSubsystem;

public class SetLeds extends CommandBase {

    LedSubsystem ledSubsystem;

    public SetLeds(LedSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        long time = System.currentTimeMillis();
        for(int i = 0; i <= 299; i++) {
            if(time % 3000 <= 1500) {
                ledSubsystem.setLed(i, 0, 50, 200);
            } else {
                ledSubsystem.setLed(i, 255, 255, 0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.subsystems.spinner.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.spinner.SpinnerSubsystem;

public class SpinnerStage3 extends CommandBase {

    SpinnerSubsystem spinnerSubsystem;

    private boolean endCondition = false;

    public SpinnerStage3(SpinnerSubsystem spinnerSubsystem) {
        this.spinnerSubsystem = spinnerSubsystem;
        addRequirements(spinnerSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        spinnerSubsystem.deploySpinner();

        char targetColor = spinnerSubsystem.getFMSColor();
                char currentColor = spinnerSubsystem.getColor();
                if (targetColor == 'R') {
                    spinnerSubsystem.setSpinnerSpeed(0.15);
                    if (currentColor == 'B') {
                        endCondition = true;
                    }
                }
                else if (targetColor == 'G') {
                    spinnerSubsystem.setSpinnerSpeed(0.15);
                    if (currentColor == 'Y') {
                        endCondition = true;
                    }
                }
                else if (targetColor == 'B') {
                    spinnerSubsystem.setSpinnerSpeed(0.15);
                    if (currentColor == 'R') {
                        endCondition = true;
                    }
                }
                else if (targetColor == 'Y') {
                    spinnerSubsystem.setSpinnerSpeed(0.15);
                    if (currentColor == 'G') {
                        endCondition = true;
                    }
                }

    }

    @Override
    public void end(boolean interrupted) {
        spinnerSubsystem.retractSpinner();
        spinnerSubsystem.setSpinnerSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return endCondition;
    }
    
}

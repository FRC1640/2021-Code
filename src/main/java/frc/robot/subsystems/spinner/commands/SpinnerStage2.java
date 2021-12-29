package frc.robot.subsystems.spinner.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.spinner.SpinnerSubsystem;

public class SpinnerStage2 extends CommandBase {

    SpinnerSubsystem spinnerSubsystem;

    private char firstColor = ' ';
    private int count = 0;
    private char prevColor= ' ';
    private char currentColor = ' ';
    private boolean endCondition = false;
    private boolean stop;

    public SpinnerStage2(SpinnerSubsystem spinnerSubsystem) {
        this.spinnerSubsystem = spinnerSubsystem;
        addRequirements(spinnerSubsystem);
    }

    @Override
    public void initialize() {
        switch (spinnerSubsystem.getColor()) {
            case 'B': {
                firstColor = 'B';
            } break;
            case 'G': {
                firstColor = 'G';
            } break;
            case 'R': {
                firstColor = 'R';
            } break;
            case 'Y': {
                firstColor = 'Y';
            }
        }
    }

    @Override
    public void execute() {

        spinnerSubsystem.deploySpinner();
        currentColor = spinnerSubsystem.getColor();

        if(firstColor != currentColor && stop == false) {
            firstColor = currentColor;
            stop = true;
        }
        if (count <= 8) {
            spinnerSubsystem.setSpinnerSpeed(1.0);
            if (currentColor == firstColor && currentColor != prevColor) {
                count++;
            }
        } else {
            endCondition = true;
        }

        prevColor = currentColor;

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

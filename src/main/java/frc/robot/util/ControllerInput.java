package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerInput extends SubsystemBase {

    public double x, y, theta;
    private Joystick controller;

    public ControllerInput(Joystick controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        x = controller.getX();
        y = controller.getY();
        theta = controller.getZ();
    }

    public double getMagnitude() {return controller.getMagnitude();}
    public double getDirection() {return controller.getDirectionDegrees();}
}

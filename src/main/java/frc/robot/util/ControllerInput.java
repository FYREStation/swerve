package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerInput extends SubsystemBase {

    public double x, y, theta;
    private Joystick controller;

    private Runnable resetFunction;

    public ControllerInput(Joystick controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        x = controller.getX();
        y = controller.getY();
        theta = controller.getZ();
        //controller.button(1, null).ifHigh(resetFunction);
    }

    public void setResetFunction(Runnable resetFunction) {this.resetFunction = resetFunction;}
    public double getMagnitude() {return controller.getMagnitude();}
    public double getDirection() {return controller.getDirectionDegrees();}
}

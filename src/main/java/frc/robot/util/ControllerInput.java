package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerInput extends SubsystemBase {

    public double x, y, theta;
    private XboxController controller;

    private Runnable resetFunction;

    public ControllerInput(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        x = controller.getLeftX();
        y = controller.getLeftY();
        if (Math.abs(x) < 0.15 && Math.abs(y) < 0.05) {
            x = 0;
            y = 0;
        }

        theta = controller.getRightX();

        //controller.button(1, null).ifHigh(resetFunction);
    }

    public void setResetFunction(Runnable resetFunction) {this.resetFunction = resetFunction;}
    public double getMagnitude() {return Math.sqrt(x * x + y * y);}
}

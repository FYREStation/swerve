package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerInput extends SubsystemBase {

    // add more to this when we get more vision stuff going
    public enum VisionStatus {
        NONE,
        ALIGN_TAG,
        LOCKON
    }

    private double x, y, theta;

    // enables / disables "full throttle" on the drive wheels
    private boolean nos;

    private boolean fieldRelative;
    private boolean alignWithTag;

    private VisionStatus visionStatus;

    private XboxController controller;

    public ControllerInput(XboxController controller) {
        this.controller = controller;
    }

    @Override
    public void periodic() {
        // controls the X and Y directions of the robot respectively
        x = controller.getLeftX();
        y = controller.getLeftY();

        // simple deadzone, we can change this to be a circle instead of a square but it doesn't really matter
        if (Math.abs(x) < 0.15 && Math.abs(y) < 0.05) {
            x = 0;
            y = 0;
        }

        // this controls the robot spinning 
        theta = controller.getRightX();

        if (Math.abs(theta) < 0.05) {
            theta = 0;
        }

        // NOS :)
        nos = controller.getRightTriggerAxis() > 0.75;

        // field relative :)
        fieldRelative = !controller.getRightBumperButton();

        // This is just a basic thing - we can make it more complex if we want for auto or smth
        alignWithTag = controller.getLeftBumperButton();

        if (controller.getLeftBumperButton()) visionStatus = VisionStatus.ALIGN_TAG;
        else if (controller.getLeftTriggerAxis() > 0.75) visionStatus = VisionStatus.LOCKON;
        else visionStatus = VisionStatus.NONE;
    }

    public double getMagnitude() {return Math.sqrt(x * x + y * y);}
    public double x() {return x;}
    public double y() {return y;}
    public double theta() {return theta;}
    public boolean nos() {return nos;}
    public boolean fieldRelative() {return fieldRelative;}
    public boolean alignWithTag() {return alignWithTag;}
    public VisionStatus visionStatus() {return visionStatus;}
}

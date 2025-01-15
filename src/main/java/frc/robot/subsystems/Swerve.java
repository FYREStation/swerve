package frc.robot.subsystems;

import frc.robot.util.Vision;
import java.util.concurrent.TimeUnit;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.util.ControllerInput;

public class Swerve extends SubsystemBase{

    private final ControllerInput controllerInput;

    private final SparkMax[] swerveMotors = new SparkMax[4];
    private final SparkMaxConfig[] swerveConfig = new SparkMaxConfig[4];

    private final SparkMax[] driveMotors = new SparkMax[4];
    private final SparkMaxConfig[] driveConfig = new SparkMaxConfig[4];

    private final RelativeEncoder[] swerveEncoders = new RelativeEncoder[4];

    private final SparkAbsoluteEncoder[] swerveEncodersAbsolute = new SparkAbsoluteEncoder[4];

    private final SparkClosedLoopController[] swervePID = new SparkClosedLoopController[4];


    private final PIDController turnPID = new PIDController(
        0.02,
        0.01,
        0.00,
        0.02
    );

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        DriverConstants.frontLeft,
        DriverConstants.frontRight,
        DriverConstants.backLeft,
        DriverConstants.backRight
    );

    double lastMotorSpeeds[] = new double[4];
    double lastMotorSetTimes[] = new double[4];

    private final Vision VisionSystem = new Vision(Constants.VisionConstants.ipAddress, Constants.VisionConstants.CameraRotations, Constants.VisionConstants.apriltagAngles); 

    private final AHRS gyroAhrs;

    private double turnTarget = 0;

    private double[] initialStates = new double[4];

    boolean setupComplete = false;

    public class SwerveAngleSpeed{
        double targetAngle;
        int multiplier;
    }

    public Swerve(ControllerInput controller) {

        controllerInput = controller;

        // define the gyro
        gyroAhrs = new AHRS(NavXComType.kMXP_SPI);
        // reset the gyro
        gyroAhrs.reset();

        // sets up the motors
        setupMotors();

    }

    @Override
    public void periodic() {
        // for (int i = 0; i < 4; i++) {
        //     System.out.printf("%d: %f\n", i, getAbsolutePosition(i));
        // }
        
        if (setupComplete) {
            if (controllerInput.alignWithTag()){
                
                ChassisSpeeds temp = VisionSystem.getTagDrive(0);
                if(temp != null){
                    swerveDrive(temp);
                }
            } else {
                swerveDrive();
            }
        } else {
            VisionSystem.clear();
            for (int i = 0; i < 4; i++) {
                if (i == 1) continue;
                /*
                system.out.printf("cur: %f - currelative: %f - offset %f\n",
                    getabsoluteposition(i),
                    swerveencoders[i].getposition(),
                    driverconstants.absoluteoffsets[i]
                );
                */
                if (Math.abs(swerveEncoders[i].getPosition() - DriverConstants.absoluteOffsets[i]) > 1.5) return;
            }
            setupComplete = true;
            resetEncoders();
            for (int i = 0; i < 4; i++) {
                swervePID[i].setReference(0, ControlType.kPosition);
            }
            try {TimeUnit.SECONDS.sleep(5);} catch (InterruptedException e) {e.getStackTrace();}
            //for (int i = 0; i < 1000000; i++) {}
        }
    }

    private void setupMotors() {

        // if this needs to loop more than 4 times, something is very wrong
        for (int i = 0; i < 4; i++) {

            swerveMotors[i] = new SparkMax(
                DriverConstants.swerveMotorPorts[i],
                SparkLowLevel.MotorType.kBrushless
            );

            driveMotors[i] = new SparkMax(
                DriverConstants.driveMotorPorts[i],
                SparkLowLevel.MotorType.kBrushless
            );
            
            swerveEncoders[i] = swerveMotors[i].getEncoder();
            swerveEncodersAbsolute[i] = swerveMotors[i].getAbsoluteEncoder();


            swerveConfig[i]
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(15);

            driveConfig[i]
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30);


            swerveConfig[i].encoder
                .positionConversionFactor(360 / 12.8)
                .positionConversionFactor(1);

            driveConfig[i].encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);


            swerveConfig[i].closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                    DriverConstants.swerveP,
                    DriverConstants.swerveI,
                    DriverConstants.swerveD,
                    DriverConstants.swerveFF
                )
                .iZone(0)
                .outputRange(-1, 1);

            swerveConfig[i].signals
                .primaryEncoderPositionPeriodMs(20);

            driveConfig[i].signals
                .primaryEncoderPositionPeriodMs(100);

            // save config into the sparks
            swerveMotors[i].configure(swerveConfig[i], ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            driveMotors[i].configure(driveConfig[i], ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


            double relativeZero = getAbsolutePosition(i);

            REVLibError error = swerveEncoders[i].setPosition(relativeZero);

            // set the swerve pid to try to reset to zero
            if (i == 1) continue;
            swervePID[i].setReference(
                DriverConstants.absoluteOffsets[i],
                SparkMax.ControlType.kPosition
            );

            if (error.equals(REVLibError.kOk)) System.out.println("Motor controller took value");

            initialStates[i] = relativeZero;

        }

        turnPID.disableContinuousInput();
        turnPID.setSetpoint(0);
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {return swerveDriveKinematics;}

    public ChassisSpeeds getRobotState() {return swerveDriveKinematics.toChassisSpeeds(getSwerveModuleState());}

    public double getAngle() {return gyroAhrs.getAngle();}

    public void resetGyro() {
        gyroAhrs.reset();
        turnTarget = 0;
    }

    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            swerveEncoders[i].setPosition(0);
            //swerveEncodersAbsolute[i].reset();
        }

    }

    double doubleMod(double x, double y) {
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x / y) * y);
    }

    public double getAbsolutePosition(int moduleNumber) {
        return 360 - (swerveEncodersAbsolute[moduleNumber].getPosition() * 360);
    }

    public SwerveModuleState[] getSwerveModuleState() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            SwerveModuleState moduleState = new SwerveModuleState(
                    (driveMotors[i].getEncoder().getVelocity() / 60d) * DriverConstants.metersPerRotation,
                    Rotation2d.fromDegrees(getAbsolutePosition(i)));
            swerveModuleState[i] = moduleState;
        }
        return swerveModuleState;
    }

    public void swerveDrive() {
        double turnSpeed = 0;
        if (Math.abs(controllerInput.theta()) < 0.01) {
            double error = turnTarget + getAngle();
            turnPID.setSetpoint(0);
            if (Math.abs(error) > 2) turnSpeed = turnPID.calculate(error);
            turnSpeed = 0;
        } else  {
            turnSpeed = controllerInput.theta(); // code orange multiplies this by 6
            turnTarget = getAngle();
        }

        if (controllerInput.fieldRelative()){
                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DriverConstants.highDriveSpeed * controllerInput.x(),
                DriverConstants.highDriveSpeed * controllerInput.y(),
                turnSpeed,
                Rotation2d.fromDegrees(getAngle())
                //Rotation2d.fromDegrees(0)
            );

            swerveDrive(chassisSpeeds);
            return;
        }
        // If we are not in field relative mode, we are in robot relative mode, so dont do the field thing
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            DriverConstants.highDriveSpeed * controllerInput.x(),
            DriverConstants.highDriveSpeed * controllerInput.y(),
            turnSpeed
        );

        swerveDrive(chassisSpeeds);
    }

    public void swerveDrive(ChassisSpeeds chassisSpeeds) {

        SwerveModuleState[] moduleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0 || chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, DriverConstants.highDriveSpeed);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState targetState = moduleState[i];
            double currentAngle = swerveEncoders[i].getPosition();
            double targetAngle = targetState.angle.getDegrees();
            SwerveAngleSpeed absoluteTarget = getAbsoluteTarget(targetAngle, currentAngle);
            
            //System.out.printf("%f, %f, %f\n", currentAngle, targetAngle, absoluteTarget);

            //System.out.println("Driving");

            if (rotate) {
                swervePID[i].setReference(absoluteTarget.targetAngle, SparkMax.ControlType.kPosition);
            }

            setMotorSpeed(i, absoluteTarget.multiplier * targetState.speedMetersPerSecond * DriverConstants.speedModifier);
            // driveMotors[i].set(
            //     controllerInput.getMagnitude() 
            //     * (controllerInput.nos() ? DriverConstants.highDriveSpeed : DriverConstants.standardDriveSpeed)
            //     * DriverConstants.speedModifier
            // );

        }
    }

    public void setMotorSpeed(int module, double velocity) {
        double time = Timer.getFPGATimestamp();
        double acceleration = time - lastMotorSetTimes[module] > 0.1
            ? 0
            : (velocity - lastMotorSpeeds[module]) / (time - lastMotorSetTimes[module]);

        double ffv = DriverConstants.driveFeedForward[module].calculateWithVelocities(velocity, acceleration);
        driveMotors[module].setVoltage(ffv);
        lastMotorSpeeds[module] = velocity;
        lastMotorSetTimes[module] = time;
    }

    /**
     * Returns the absolute angle a module needs to approach
     * @param targetAngle - the angle the module should be at
     * @param currentAngle - the current position of the module
     * @return absoluteTarget - the absolute angle the module needs to approach
     */

    
    private SwerveAngleSpeed getAbsoluteTarget(double targetAngle, double currentAngle) {

        targetAngle += 180;
        int multiplier = 1;

        double angleDiff = targetAngle - doubleMod(doubleMod(currentAngle, 360) + 360, 360);

        if (angleDiff > 180) {
            angleDiff -= 360;
        } else if (angleDiff < -180) {
            angleDiff += 360;
        }

        if (angleDiff < -90){
            angleDiff += 180;
            multiplier = -1;
        } else if (angleDiff > 90){
            angleDiff -= 180;
            multiplier = -1;
        }

        SwerveAngleSpeed returnThing = new SwerveAngleSpeed();
        returnThing.multiplier = multiplier;
        returnThing.targetAngle = currentAngle + angleDiff;

        return returnThing;
    }

}

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.util.ControllerInput;

public class Swerve extends SubsystemBase{

    private final ControllerInput controllerInput;

    private final CANSparkMax[] swerveMotors = new CANSparkMax[4];

    private final CANSparkMax[] driveMotors = new CANSparkMax[4];

    private final RelativeEncoder[] swerveEncoders = new RelativeEncoder[4];

    private final DutyCycleEncoder[] swerveEncodersDIO = new DutyCycleEncoder[4];

    private final SparkPIDController[] swervePID = new SparkPIDController[4];

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

    private final AHRS gyroAhrs;

    private final Rotation2d wantedHeading = new Rotation2d();

    private double turnTarget = 0;

    private int turnOverFlowCount = 0;

    private double[] moduleTurnStates = {
        0, 0, 0, 0
    };

    private double[] initialStates = new double[4];

    boolean setupComplete = false;

    public Swerve(ControllerInput controller) {

        controllerInput = controller;

        // define the gyro
        gyroAhrs = new AHRS(SPI.Port.kMXP);
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
        if (setupComplete) swerveDrive();
        else {
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

        try {TimeUnit.SECONDS.sleep(1);} catch (InterruptedException e) {e.printStackTrace();}

        // if this needs to loop more than 4 times, something is very wrong
        for (int i = 0; i < 4; i++) {
            swerveMotors[i] = new CANSparkMax(
                DriverConstants.swerveMotorPorts[i],
                CANSparkLowLevel.MotorType.kBrushless
            );

            driveMotors[i] = new CANSparkMax(
                DriverConstants.driveMotorPorts[i],
                CANSparkLowLevel.MotorType.kBrushless
            );

            swerveEncoders[i] = swerveMotors[i].getEncoder();
            swerveEncoders[i].setPositionConversionFactor(360 / 12.8); // this is arbitrary
            driveMotors[i].getEncoder().setPositionConversionFactor(1);
            driveMotors[i].getEncoder().setVelocityConversionFactor(1);

            //swerveMotors[i].getAnalog(SparkAnalogSensor.Mode.kAbsolute).setPositionConversionFactor(360 / 3.3); // this is arbitrary

            swervePID[i] = swerveMotors[i].getPIDController();
            swervePID[i].setP(DriverConstants.swerveP);
            swervePID[i].setI(DriverConstants.swerveI);
            swervePID[i].setD(DriverConstants.swerveD);
            swervePID[i].setFF(DriverConstants.swerveFF);
            swervePID[i].setIZone(0);
            swervePID[i].setOutputRange(-1, 1);


            driveMotors[i].setInverted(false);
            swerveMotors[i].setInverted(true);
            
            // TODO: check this
            //swerveMotors[i].isFollower();
            swerveEncodersDIO[i] = new DutyCycleEncoder(DriverConstants.encoders[i]);

            // get data faster from the sparks
            swerveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

            driveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

            swerveMotors[i].setSmartCurrentLimit(15);
            driveMotors[i].setSmartCurrentLimit(30);

            driveMotors[i].setIdleMode(IdleMode.kCoast);
            swerveMotors[i].setIdleMode(IdleMode.kBrake);

            // save config into the sparks
            driveMotors[i].burnFlash();
            //swerveMotors[i].burnFlash();

            double relativeZero = getAbsolutePosition(i);
            //if (relativeZero + getAbsolutePosition(i) < 0) {relativeZero += 360;}


            //swerveEncoders[i].setPosition(0); 

            REVLibError error = swerveEncoders[i].setPosition(relativeZero);

             // set the swerve pid to try to reset to zero
            if (i == 1) continue;
            swervePID[i].setReference(
                DriverConstants.absoluteOffsets[i],
                CANSparkMax.ControlType.kPosition
            );

            if (error.equals(REVLibError.kOk)) System.out.println("Motor controller took value");

            initialStates[i] = relativeZero;

        }

        turnPID.disableContinuousInput();
        turnPID.setSetpoint(0);

        
        // int penis = 0;
        // while (penis != Integer.MAX_VALUE) penis++;

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
            //swerveEncodersDIO[i].reset();
        }

    }

    double doubleMod(double x, double y) {
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x / y) * y);
    }

    public double getAbsolutePosition(int moduleNumber) {
        return 360 - (swerveEncodersDIO[moduleNumber].getDistance() * 360);
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

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            DriverConstants.highDriveSpeed * controllerInput.x(),
            DriverConstants.highDriveSpeed * controllerInput.y(),
            turnSpeed,
            Rotation2d.fromDegrees(getAngle())
            //Rotation2d.fromDegrees(0)
        );

        swerveDrive(chassisSpeeds);
    }

    public void swerveDrive(ChassisSpeeds chassisSpeeds) {

        SwerveModuleState[] moduleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0 || chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, DriverConstants.highDriveSpeed);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState targetState = moduleState[i];
            double targetAngle = targetState.angle.getDegrees();
            double currentAngle = swerveEncoders[i].getPosition();

            double absoluteTarget = getAbsoluteTarget(targetAngle, currentAngle);

            //System.out.printf("%f, %f, %f\n", currentAngle, targetAngle, absoluteTarget);

            //System.out.println("Driving");

            if (rotate) {
                swervePID[i].setReference(absoluteTarget, CANSparkMax.ControlType.kPosition);
            }

            setMotorSpeed(i, targetState.speedMetersPerSecond * DriverConstants.speedModifier);
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

        double ffv = DriverConstants.driveFeedForward[module].calculate(velocity, acceleration);
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
    private double getAbsoluteTarget(double targetAngle, double currentAngle) {

        targetAngle += 180;

        double angleDiff = targetAngle - doubleMod(doubleMod(currentAngle, 360) + 360, 360);

        if (angleDiff > 180) {
            angleDiff -= 360;
        } else if (angleDiff < -180) {
            angleDiff += 360;
        }

        return currentAngle + angleDiff;
    }

}

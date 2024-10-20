package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;

public class Swerve extends SubsystemBase{

    private final CANSparkMax[] swerveMotors = new CANSparkMax[4];

    private final CANSparkMax[] driveMotors = new CANSparkMax[4];

    private final RelativeEncoder[] swerveEncoders = new RelativeEncoder[4];

    private final Encoder[] swerveEncodersDIO = new Encoder[4];

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

    public Swerve() {
        // sets up the motors
        setupMotors();

        // define the gyro
        gyroAhrs = new AHRS(SPI.Port.kMXP);
        // reset the gyro
        gyroAhrs.reset();

    }

    private void setupMotors() {
        // if this needs to loop more than 4 times, something is very wrong
        for (int i = 0; i < 4; i++) {
            swerveMotors[i] = new CANSparkMax(
                DriverConstants.swerveMotorPorts[i],
                CANSparkLowLevel.MotorType.kBrushless
            );
            swerveEncoders[i] = swerveMotors[i].getEncoder();
            swerveEncoders[i].setPositionConversionFactor(8.1503); // this is arbitrary
            driveMotors[i].getEncoder().setPositionConversionFactor(1);
            driveMotors[i].getEncoder().setVelocityConversionFactor(1);

            swerveMotors[i].getAnalog(SparkAnalogSensor.Mode.kAbsolute).setPositionConversionFactor(360 / 3.3); // this is arbitrary

            swervePID[i] = swerveMotors[i].getPIDController();
            swervePID[i].setP(DriverConstants.swerveP);
            swervePID[i].setI(DriverConstants.swerveI);
            swervePID[i].setD(DriverConstants.swerveD);
            swervePID[i].setFF(DriverConstants.swerveFF);

            driveMotors[i] = new CANSparkMax(
                DriverConstants.driveMotorPorts[i],
                CANSparkLowLevel.MotorType.kBrushless
            );
            driveMotors[i].setInverted(false);

            swerveEncodersDIO[i] = new Encoder(
                DriverConstants.encoders[i][0],
                DriverConstants.encoders[i][1] 
            );


            // get data faster from the sparks
            swerveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
            driveMotors[i].setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

            swerveMotors[i].setSmartCurrentLimit(15);
            driveMotors[i].setSmartCurrentLimit(30);

            driveMotors[i].setIdleMode(IdleMode.kBrake);
            swerveMotors[i].setIdleMode(IdleMode.kBrake);

            // save config into the sparks
            driveMotors[i].burnFlash();
            swerveMotors[i].burnFlash();
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

    double doubleMod(double x, double y) {
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x / y) * y);
    }

    public double getAbsolutePosition(int moduleNumber) {
        double angle = ((1 - swerveEncodersDIO[moduleNumber].get()) * 360) - 90;
        return angle < 0 ? angle + 360 : angle;
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

    public void swerveDrive(double velocityX, double velocityY, double theta) {
        double turnSpeed = 0;
        if (Math.abs(theta) < 0.01) {
            double error = turnTarget + getAngle();
            turnPID.setSetpoint(0);
            if (Math.abs(error) > 2) turnSpeed = turnPID.calculate(error);
            turnSpeed = 0;
        } else  {
            turnSpeed = theta * 6;
            turnTarget = getAngle();
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            DriverConstants.highDriveSpeed * velocityX,
            DriverConstants.highDriveSpeed * velocityY,
            turnSpeed,
            Rotation2d.fromRadians(getAngle())
        );

        swerveDrive(chassisSpeeds);
    }

    public void swerveDrive(ChassisSpeeds chassisSpeeds) {

        SwerveModuleState[] moduleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0 || chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, DriverConstants.highDriveSpeed);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState targetState = moduleState[i];
            double targetAngle = targetState.angle.getRadians();
            double currentAngle = getAbsolutePosition(i);

            double angleDiff = doubleMod((targetAngle - currentAngle) + 180, 360) - 180;

            if (Math.abs(angleDiff) < 5 || !rotate) {
                swerveMotors[i].set(0);
            } else {
                swervePID[i].setReference(swerveEncoders[i].getPosition() + angleDiff, CANSparkMax.ControlType.kPosition);
            }

            setMotorSpeed(i, targetState.speedMetersPerSecond * DriverConstants.speedModifier);

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

}

package lynxlib.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import lynxlib.util.CheesyPID;
import lynxlib.util.SetpointRamp;
import lynxlib.util.exceptions.RequiredControllerNotPresentException;

import static java.util.Objects.requireNonNull;

public class LynxDriveBase {

    Encoder leftEncoder;
    Encoder rightEncoder;

    CheesyPID leftVelocityPID;
    CheesyPID rightVelocityPID;

    Gyro gyro;
    CheesyPID gyroPID;

    double kV;
    double kS;

    double kDeadBand = 0.1;
    double kMaxOutput = 0.8;
    double kMaxSpeed = 1;
    double kGyroTurnThreshold = 0.1;
    double kTankDriveGyroTolerance = 0.1;

    SetpointRamp leftRamp = new SetpointRamp();
    SetpointRamp rightRamp = new SetpointRamp();

    boolean gyroStabilizationEnabled = false;
    boolean closedLoopControlEnabled = false;

    LynxDriveBase(){

    }


    /**
     * Sets the left and right encoder
     * Distance per pulse and inversion should be set for the object before passing it in
     *
     * @param leftEncoder  leftside drivetrain Encoder
     * @param rightEncoder rightside drivetrain Encoder
     */
    public void setEncoders(Encoder leftEncoder, Encoder rightEncoder) {
        requireNonNull(leftEncoder);
        requireNonNull(rightEncoder);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    /**
     * Sets the Left and Right drivetrain Velocity PID's
     *
     * @param leftPID  left drivetrain Velocity PID
     * @param rightPID right drivetrain Velocity PID
     */
    public void setVelocityPIDs(CheesyPID leftPID, CheesyPID rightPID) {
        requireNonNull(leftPID);
        requireNonNull(rightPID);
        leftVelocityPID = leftPID;
        rightVelocityPID = rightPID;
    }

    public void setMaxChangePerSecond(double changePerSecond) {
        leftRamp.setMaxChangePerSecond(changePerSecond);
        rightRamp.setMaxChangePerSecond(changePerSecond);
    }

    /**
     * Sets the Gyro Object to be used for Gyro Stabilization
     *
     * @param gyro the gyro object
     */
    public void setGyro(Gyro gyro) {
        requireNonNull(gyro);
        this.gyro = gyro;
    }

    /**
     * Sets the Gyro PID to be used for Gyro Stabilization
     *
     * @param gyroPID the Gyro PID
     */
    public void setGyroPID(CheesyPID gyroPID) {
        requireNonNull(gyroPID);
        this.gyroPID = gyroPID;
        this.gyroPID.setContinuous(true);
        this.gyroPID.setInputRange(0, 360);
    }

    /**
     * Sets the max speed to be scaled to when using closed loop control
     *
     * @param speed max speed - same unit as encoder distance per pulse
     */
    public void setMaxSpeed(double speed) {
        kMaxSpeed = speed;
    }

    /**
     * Sets the constants to be used in the feedforward calculations
     *
     * @param kV         kV
     * @param vIntercept vIntercept
     */
    public void setFeedForwardConstants(double kV, double vIntercept) {
        this.kV = kV;
        this.kS = vIntercept;
    }

    /**
     * Sets the tolerance for gyro stabilization in tank drive
     *
     * @param tolerance the tolerance
     */
    public void setkTankDriveGyroTolerance(double tolerance) {
        kTankDriveGyroTolerance = tolerance;
    }

    /**
     * Sets the joystick deadband
     *
     * @param deadBand the deadband
     */
    public void setkDeadBand(double deadBand) {
        kDeadBand = deadBand;
    }

    /**
     * Sets the threshold for how much gyro turn counts as actually turning
     *
     * @param threshold the threshold
     */
    public void setkGyroTurnThreshold(double threshold) {
        kGyroTurnThreshold = threshold;
    }

    /**
     * Sets the state of closedLoopControl
     * Only can be enabled after passing in both side PID's and Encoders
     *
     * @param enabled state of closed loop control
     */
    public void setClosedLoopEnabled(boolean enabled) {
        if (enabled) {
            verifyClosedLoopComponents();
        }
        closedLoopControlEnabled = enabled;
    }

    /**
     * Sets the state  of Gyro Stabilization
     * Can only be enabled after passing in both a Gyro PID and Gyro Object
     *
     * @param enabled state of gyro stabilization
     */
    public void setGyroStabilizationEnabled(boolean enabled) {
        if (enabled) {
            verifyGyroStabilizationComponents();
        }
        gyroStabilizationEnabled = enabled;
    }

    /**
     * Verifies if components for closed loop control are set
     * Throws RequiredControllerNotPresentException if one is missing
     */
    private void verifyClosedLoopComponents() {
        if (leftVelocityPID == null || rightVelocityPID == null ||
                leftEncoder == null || rightEncoder == null) {
            throw new RequiredControllerNotPresentException(
                    "Component for Closed Loop Control Missing \n" +
                            "Set Encoders and Velocity PID Objects"
            );
        }

    }

    /**
     * Verifies if components for gyro stabilization is set
     * Throws RequiredControllerNotPresentException if one is missing
     */
    private void verifyGyroStabilizationComponents() {
        if (gyro == null || gyroPID == null) {
            throw new RequiredControllerNotPresentException(
                    "Component for Gyro Stabilization Missing \n" +
                            "Set Gyro and GyroPID objects"
            );
        }

    }
}

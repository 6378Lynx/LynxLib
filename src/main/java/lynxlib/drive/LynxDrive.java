package lynxlib.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import lynxlib.util.CheesyPID;
import lynxlib.util.exceptions.RequiredControllerNotPresentException;

import java.util.function.Function;

import static java.util.Objects.requireNonNull;

public class LynxDrive {

    private final SpeedController leftMotor;
    private final SpeedController rightMotor;

    private Encoder leftEncoder;
    private Encoder rightEncoder;

    private CheesyPID leftVelocityPID;
    private CheesyPID rightVelocityPID;

    private Gyro gyro;
    private CheesyPID gyroPID;

    private boolean gyroStabilizationEnabled;
    private boolean closedLoopControlEnabled;

    private Function<Double, Double> curve = Function.identity();
    private Function<Double, Double> fwdScale = x -> Math.copySign(x * x, x);
    private Function<Double, Double> rotScale = x -> Math.copySign(x * x, x);

    private double kV;
    private double kS;

    private double kDeadBand = 0.1;
    private double kMaxOutput = 0.8;
    private double kMaxSpeed = 1;

    /**
     * Constuct a LynxDrive
     *
     * @param leftMotor  SpeedController left motor, SpeedControllerGroup for more than one motor
     * @param rightMotor SpeedController right motor, SpeedControllerGroup for more than one motor
     */
    public LynxDrive(SpeedController leftMotor, SpeedController rightMotor) {
        requireNonNull(leftMotor);
        requireNonNull(rightMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
    }

    /**
     * Arcade drive method using LynxDrive
     *
     * @param forward  robot's speed along the X axis, forward is positive
     * @param rotation robot's rotation, clockwise is positive
     */
    public void arcadeDrive(double forward, double rotation) {

        forward = scaleJoystickInput(forward, fwdScale);
        rotation = scaleJoystickInput(rotation, rotScale);

        double left = forward + rotation;
        double right = forward - rotation;

        setOutput(left, right);
    }

    /**
     * Curvature drive method using LynxDrive
     *
     * @param forward   robot's speed along the x-axis
     * @param rotation  robot's rotation, gets applied along a curvature radius
     * @param quickTurn If true, overrides curvature and allows for arcade type movement
     */
    public void curvatureDrive(double forward, double rotation, boolean quickTurn) {
        double left;
        double right;
        forward = scaleJoystickInput(forward, fwdScale);
        rotation = scaleJoystickInput(rotation, rotScale);


        if (forward == 0) {
            quickTurn = true;
        }

        if (quickTurn) {
            left = forward + rotation;
            right = forward - rotation;
        } else {
            left = forward + curve.apply(forward) * rotation;
            right = forward - curve.apply(forward) * rotation;
        }

        double[] normalSpeed = normalizeSpeeds(left, right);

        left = normalSpeed[0];
        right = normalSpeed[1];

        setOutput(left, right);
    }

    /**
     * Tank drive method using LynxDrive
     *
     * @param left
     * @param right
     */
    public void tankDrive(double left, double right) {
        left = scaleJoystickInput(left, fwdScale);
        right = scaleJoystickInput(right, fwdScale);

        setOutput(left, right);
    }

    /**
     * Sets the motors to the outputs
     *
     * @param left  the input given to the left motor
     * @param right the input given to the right motor
     */
    private void setOutput(double left, double right) {

        if(closedLoopControlEnabled) {
            left = scaleMaxSpeed(left);
            right = scaleMaxSpeed(right);

            leftVelocityPID.setSetpoint(left);
            rightVelocityPID.setSetpoint(right);

            left = leftVelocityPID.calculate(leftEncoder.getRate(), 0.02);
            right = leftVelocityPID.calculate(leftEncoder.getRate(), 0.02);

        }

        left = Math.max(-kMaxOutput, Math.min(kMaxOutput, left));
        right = Math.max(-kMaxOutput, Math.min(kMaxOutput, right));

        leftMotor.set(left);
        rightMotor.set(right);
    }

    /**
     * Calculates a the fractional out with kV and V-Intercept
     *
     * @param speed speed
     * @return fractional out to be set by a motor
     */
    private double calculateFractionalFeedForward(double speed) {
        if (speed == 0) {
            return 0;
        } else {
            return (speed * kV + Math.copySign(kS, speed)) / 12;
        }
    }


    /**
     * Prepares user joystick input to be given to the drive methods
     * Deadbands and clamps to [-maxOutput, maxOutput], scales by user-specified appropriate scaling functions
     *
     * @param input the input to be scaled
     * @param scale the scaling function used to scale input
     * @return the scaled input
     */
    private double scaleJoystickInput(double input, Function<Double, Double> scale) {
        if (Math.abs(input) < kDeadBand) {
            input = 0;
        } else {
            input = Math.max(-kMaxOutput, Math.min(kMaxOutput, input));
            input = scale.apply(input);
        }
        return input;
    }

    /**
     * Scales the input by a user-specified max speed to be used for closed loop control
     *
     * @param input the input
     * @return the scaled input
     */
    private double scaleMaxSpeed(double input) {
        return input * kMaxSpeed;
    }

    /**
     * Normalizes the speed when using Curvature drive
     *
     * @param left  the left speed
     * @param right the right speed
     * @return normalized left and right speed
     */
    private double[] normalizeSpeeds(double left, double right) {
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
        if (maxMagnitude > 1) {
            left /= maxMagnitude;
            right /= maxMagnitude;
        }
        return new double[]{left, right};
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
        this.leftVelocityPID = leftPID;
        this.rightVelocityPID = rightPID;
    }

    /**
     * Sets the Gyro Object
     *
     * @param gyro
     */
    public void setGyro(Gyro gyro) {
        requireNonNull(gyro);
        this.gyro = gyro;
    }

    /**
     * Sets the Gyro PID
     *
     * @param gyroPID the Gyro PID
     */
    public void setGyroPID(CheesyPID gyroPID) {
        requireNonNull(gyroPID);
        this.gyroPID = gyroPID;
    }

    /**
     * Sets the state of closedLoopControl
     * Only can be enabled after passing in both side PID's and Encoders
     *
     * @param enabled
     */
    public void setClosedLoopEnabled(boolean enabled) {
        if (enabled) {
            verifyClosedLoopComponents();
        }
        this.closedLoopControlEnabled = enabled;
    }

    /**
     * Sets the state  of Gyro Stabilization
     * Can only be enabled after passing in both a Gyro PID and Gyro Object
     *
     * @param enabled
     */
    public void setGyroStabilizationEnabled(boolean enabled) {
        if (enabled) {
            verifyGyroStabilizationComponents();
        }
        this.gyroStabilizationEnabled = enabled;
    }

    /**
     * Verifies if components for closed loop control are set
     * Throws RequiredControllerNotPresentException if one is missing
     */
    private void verifyClosedLoopComponents() {
        if (this.leftVelocityPID == null || this.rightVelocityPID == null ||
                this.leftEncoder == null || this.rightEncoder == null) {
            throw new RequiredControllerNotPresentException(
                    "Component for Closed Loop Control Missing \n" +
                            "Set Encoders and Velocity PID Objects"
            );
        }

    }

    /**
     * Verifies if components for gyro stabilization is set
     * Throwws RequiredControllerNotPresentException if one is missing
     */
    private void verifyGyroStabilizationComponents() {
        if (this.gyro == null || this.gyroPID == null) {
            throw new RequiredControllerNotPresentException(
                    "Component for Gyro Stabilization Missing \n" +
                            "Set Gyro and GyroPID objects"
            );
        }

    }

}

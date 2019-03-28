package lynxlib.util.exceptions;

/**
 * This exception will be thrown if a controller is enabled without the objects required for the controller.
 * <p>
 * Can be caused if you don't set the requirements for the controller before enabling it
 */
public class RequiredControllerNotPresentException extends RuntimeException {
    /**
     * Creates a new RequiredControllerNotPresentException
     */
    public RequiredControllerNotPresentException() {
    }

    /**
     * Creates a new RequiredControllerNotPresentException with the given message
     *
     * @param message the message
     */
    public RequiredControllerNotPresentException(String message) {
        super(message);
    }

}

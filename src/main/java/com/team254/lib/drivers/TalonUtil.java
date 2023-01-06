package com.team254.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonUtil {
    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkError(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            DriverStation.reportError(message + " " + errorCode, false);
        }
    }

    /**
     * checks the specified error code and throws an exception if there are any issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkErrorWithThrow(ErrorCode errorCode, String message) {
        if (errorCode != ErrorCode.OK) {
            throw new RuntimeException(message + " " + errorCode);
        }
    }
}

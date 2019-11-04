package org.firstinspires.ftc.teamcode.SkystoneTeleOp;

/*
 * Container that holds variables that are referenced in multiple classes
 *
 * */
public class RobotState {
    private static boolean isManual = false; // whether it is in manual control or automatic (run to target position)
    private static boolean isCurrentlyMoving = false; // whether or not it is going to a target position

    // START OF GETTERS AND SETTERS

    public static boolean getIsManual() {
        return isManual;
    }

    public static void setIsManual(boolean isManual) {
        RobotState.isManual = isManual;
    }

    /*
     * return the variable isCurrentlyMoving
     * */
    public static boolean isCurrentlyMoving() {
        return isCurrentlyMoving;
    }

    public static void setCurrentlyMoving(boolean moving) {
        isCurrentlyMoving = moving;
    }


}

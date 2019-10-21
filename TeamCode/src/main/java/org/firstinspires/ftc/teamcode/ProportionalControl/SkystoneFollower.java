package org.firstinspires.ftc.teamcode.ProportionalControl;

public class SkystoneFollower {

    private double strafeFactor = 0.04, fwdFactor = 0.02;
    /*---------------------------------------------------------------------------------------------*/
    private double xTolerance = 2.5, yTolerance = 2.5, xTarget = 0.0, yTarget = 0.0;
    private boolean withinXTolerance = false, withinYTolerance = false;

    /**
     * @param xOffset
     * @param yOffset
     * @return strafe and fwd powers as array
     */
    public double[] calculateOutput(double xOffset, double yOffset) {
        return calculateOutput(xOffset, yOffset, xTarget, yTarget, xTolerance, yTolerance);
    }

    public double[] calculateOutput(double xOffset, double yOffset, double xTarget, double yTarget) {
        return calculateOutput(xOffset, yOffset, xTarget, yTarget, xTolerance, yTolerance);
    }

    /**
     * @param xOffset
     * @param yOffset
     * @param xTarget
     * @param yTarget
     * @param xTolerance
     * @param yTolerance
     * @return strafe and fwd output as array
     */
    public double[] calculateOutput(double xOffset, double yOffset, double xTarget, double yTarget, double xTolerance, double yTolerance) {
        double xError = (xTarget - xOffset);
        double yError = -(yTarget - yOffset);

        withinXTolerance = Math.abs(xError) <= xTolerance;
        withinYTolerance = Math.abs(yError) <= yTolerance;

        double forward = xError * fwdFactor;
        double strafe = yError * strafeFactor;

        return new double[]{strafe, forward};
    }

    public boolean withinXTolerance() {
        return withinXTolerance;
    }

    public boolean withinYTolerance() {
        return withinYTolerance;
    }

    public boolean withinTolerance() {
        return withinYTolerance() && withinXTolerance();
    }

    public double getYTolerance() {
        return yTolerance;
    }

    public void setYTolerance(double yTolerance) {
        this.yTolerance = yTolerance;
    }

    public double getYTarget() {
        return yTarget;
    }

    public void setYTarget(double yTarget) {
        this.yTarget = yTarget;
    }

    public double getXTarget() {
        return xTarget;
    }

    public void setXTarget(double xTarget) {
        this.xTarget = xTarget;
    }

    public double getXTolerance() {
        return xTolerance;
    }

    public void setXTolerance(double xTolerance) {
        this.xTolerance = xTolerance;
    }

    public double getFwdFactor() {
        return fwdFactor;
    }

    public void setFwdFactor(double fwdFactor) {
        this.fwdFactor = fwdFactor;
    }

    public double getStrafeFactor() {
        return strafeFactor;
    }

    public void setStrafeFactor(double strafeFactor) {
        this.strafeFactor = strafeFactor;
    }
}

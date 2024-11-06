/**
 * A Simplified Analytic Attitude Determination Algorithm
 * using Accelerometer and Magnetometer on micro:bit.
 * 
 * It's like magic! This algorithm turns raw data into accurate and efficient quaternion estimations,
 * transforming your projects and making you go, "Wow!" Using the handy micro:bit, it feels like trying
 * out quaternions is pure magic. AccelMagiQ brings a touch of enchantment to the technical world.
 * We hope this helps you in your learning journey and sparks your curiosity about the fascinating
 * world of quaternions.
 * 
 * "AccelMagiQ" refers to this amazing algorithm that combines accelerometer and magnetometer data to
 * create precise quaternion calculations. It's a playful blend of 'acceleration', 'magnetometer', and 'magic' with
 * quaternions ('Q'), making advanced concepts seem like magic!
 */
//% block="AccelMagiQ"
//% weight=100 color=#696969 icon="\uf1b2"
//% groups="['Quaternion', 'EulerAngles', 'Sensor']"
namespace accelmagiq {

    /** 
     * Define the EstimationMethod enum
     */
    export enum EstimationMethod {
        /**
         * FAMC estimation method
         */
        //% block="FAMC(Default)"
        FAMC = 0,
        /**
         * SIMPLE estimation method
         */
        //% block="Simple"
        SIMPLE = 1
    }

    /**
     * Updates the acceleration values.
     * @param x X-axis acceleration
     * @param y Y-axis acceleration
     * @param z Z-axis acceleration
     */
    //% block="Update Acceleration X %x Y %y Z %z"
    //% group="Sensor"
    //% weight=143
    export function updateAcc(x: number, y: number, z: number): void {
        accelmagiq_.updateAcceleration(x, y, z);
    }

    /**
     * Updates the magnetic force values.
     * @param x X-axis magnetic force
     * @param y Y-axis magnetic force
     * @param z Z-axis magnetic force
     */
    //% block="Update Magnetic force X %x Y %y Z %z"
    //% group="Sensor"
    //% weight=142
    export function updateMag(x: number, y: number, z: number): void {
        accelmagiq_.updateMagneticForce(x, y, z);
    }

    /**
     * Estimates the current quaternion.
     * @returns An array containing the quaternion components [w, x, y, z].
     */
    //% block="Estimate Quaternion"
    //% group="Sensor"
    //% weight=141
    export function estimate(): number[] {
        accelmagiq_.estimate();
        return [accelmagiq_.getW(), accelmagiq_.getX(), accelmagiq_.getY(), accelmagiq_.getZ()];
    }

    /**
     * Sets the alpha value for the low-pass filter.
     * @param alpha The new alpha value (between 0.0 and 1.0). Default is 0.8.
     */
    //% block="Set Alpha %alpha"
    //% group="Sensor"
    //% weight=140
    //% alpha.defl=0.8
    //% advanced=true
    export function setAlpha(alpha: number): void {
        accelmagiq_.setLowPassFilterAlpha(alpha);
    }

    /**
     * Sets the estimation method.
     */
    //% block="set estimate method %method"
    //% group="Sensor"
    //% weight=130
    //% method.defl=0
    //% advanced=true
    export function setEstimateMethod(method: EstimationMethod): void {
        accelmagiq_.setEstimateMethod(method);
    }

}

namespace accelmagiq {

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
        accelmagiqfamc_.updateAcceleration(x, y, z);
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
        accelmagiqfamc_.updateMagneticForce(x, y, z);
    }

    /**
     * Estimates the current quaternion.
     * @returns An array containing the quaternion components [w, x, y, z].
     */
    //% block="Estimate Quaternion"
    //% group="Sensor"
    //% weight=141
    export function estimate(): number[] {
        accelmagiqfamc_.estimate();
        return [accelmagiqfamc_.getW(), accelmagiqfamc_.getX(), accelmagiqfamc_.getY(), accelmagiqfamc_.getZ()];
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
    export function setAlpha(alpha: number = 0.8): void {
        accelmagiqfamc_.setLowPassFilterAlpha(alpha);
    }

}

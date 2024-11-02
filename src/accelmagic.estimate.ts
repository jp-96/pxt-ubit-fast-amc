namespace accelmagic {

    //% block
    //% group="Sensor"
    //% weight=130
    export function updateAcc(x: number, y: number, z: number): void {
        accelmagicfamc_.updateAcceleration(x, y, z);
    }

    //% block
    //% group="Sensor"
    //% weight=120
    export function updateMag(x: number, y: number, z: number): void {
        accelmagicfamc_.updateMagneticForce(x, y, z);
    }

    //% block
    //% group="Sensor"
    //% weight=110
    export function estimate(): number[] {
        accelmagicfamc_.estimate();
        return [accelmagicfamc_.getW(), accelmagicfamc_.getX(), accelmagicfamc_.getY(), accelmagicfamc_.getZ()];
    }

    //% block
    //% group="Sensor"
    //% weight=100
    //% advanced=true
    export function setAlpha(alpha: number): void {
        accelmagicfamc_.setLowPassFilterAlpha(alpha);
    }

}

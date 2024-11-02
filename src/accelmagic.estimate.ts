namespace accelmagic {

    //% block
    //% group="Sensor"
    //% weight=130
    export function updateAcc(x: number, y: number, z: number): void {
        famc_.updateAcceleration(x, y, z);
    }

    //% block
    //% group="Sensor"
    //% weight=120
    export function updateMag(x: number, y: number, z: number): void {
        famc_.updateMagneticForce(x, y, z);
    }

    //% block
    //% group="Sensor"
    //% weight=110
    export function estimate(): Quaternion {
        famc_.estimate();
        return new Quaternion(famc_.getW(), famc_.getX(), famc_.getY(), famc_.getZ());
    }

    //% block
    //% group="Sensor"
    //% weight=100
    //% advanced=true
    export function setAlpha(alpha: number): void {
        famc_.setLowPassFilterAlpha(alpha);
    }

}

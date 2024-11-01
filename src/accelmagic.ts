/**
 * A Simplified Analytic Attitude Determination Algorithm
 * Using Accelerometer and Magnetometer on micro:bit.
 * 
 */
//% block="Accel Magic"
//% weight=100 color=#696969 icon="\uf1b2"
namespace accelmagic {

    const DEFAULT_ALPHA = 0.8;
    setLowPassFilterAlpha(DEFAULT_ALPHA);

    //% block
    export function updateAcceleration(x: number, y: number, z: number): void {
        famc_.updateAcceleration(x, y, z);
    }

    //% block
    export function updateMagneticForce(x: number, y: number, z: number): void {
        famc_.updateMagneticForce(x, y, z);
    }

    //% block
    export function estimateQuaternion(): number[] {
        famc_.estimate();
        return [famc_.getW(), famc_.getX(), famc_.getY(), famc_.getZ()];
    }

    //% block
    export function quaternionToEulerAngles(quaternion: number[]): number[] {
        const q = Quaternion.normalize(quaternion);
        const qw = q[0];
        const qx = q[1];
        const qy = q[2];
        const qz = q[3];

        const ysqr = qy * qy;
        const t0 = 2.0 * (qw * qx + qy * qz);
        const t1 = 1.0 - 2.0 * (qx * qx + ysqr);
        const bank = Math.atan2(t0, t1);

        let t2 = 2.0 * (qw * qy - qz * qx);
        t2 = t2 > 1.0 ? 1.0 : (t2 < -1.0 ? -1.0 : t2);
        const pitch = Math.asin(t2);

        const t3 = 2.0 * (qw * qz + qx * qy);
        const t4 = 1.0 - 2.0 * (ysqr + qz * qz);
        const heading = Math.atan2(t3, t4);

        return [heading, pitch, bank];
    }

    const C180_OVER_PI = 180 / Math.PI;

    function radToDeg(rad: number): number {
        return Math.round(rad * C180_OVER_PI);
    }

    //% block
    export function getHeading(eulerAngles: number[]): number {
        return radToDeg(getHeadingRadians(eulerAngles));
    }

    //% block
    export function getPitch(eulerAngles: number[]): number {
        return radToDeg(getPitchRadians(eulerAngles));
    }

    //% block
    export function getBank(eulerAngles: number[]): number {
        return radToDeg(getBankRadians(eulerAngles));
    }

    //% block
    export function getAzimuth(eulerAngles: number[]): number {
        return radToDeg(getAzimuthRadians(eulerAngles));
    }

    //% block
    //% advanced=true
    export function getHeadingRadians(eulerAngles: number[]): number {
        return eulerAngles[0];
    }

    //% block
    //% advanced=true
    export function getPitchRadians(eulerAngles: number[]): number {
        return eulerAngles[1];
    }
    //% block
    //% advanced=true
    export function getBankRadians(eulerAngles: number[]): number {
        return eulerAngles[2];
    }

    const C_PI_2 = Math.PI * 2.0;

    //% block
    //% advanced=true
    export function getAzimuthRadians(eulerAngles: number[]): number {
        const heading = getHeadingRadians(eulerAngles);
        if (heading > 0) {
            return C_PI_2 - heading;
        } else {
            return -1.0 * heading;
        }
    }

    //% block
    //% advanced=true
    export function createQuaternion(w: number, x: number, y: number, z: number): number[] {
        return Quaternion.normalize([w, x, y, z]);
    }

    //% block
    //% advanced=true
    export function quaternionW(q: number[]): number {
        return q[0];
    }

    //% block
    //% advanced=true
    export function quaternionX(q: number[]): number {
        return q[1];
    }

    //% block
    //% advanced=true
    export function quaternionY(q: number[]): number {
        return q[2];
    }

    //% block
    //% advanced=true
    export function quaternionZ(q: number[]): number {
        return q[3];
    }

    //% block
    //% advanced=true
    export function normalize(q: number[]): number[] {
        return Quaternion.normalize(q);
    }

    //% block
    //% advanced=true
    export function conjugate(q: number[]): number[] {
        return Quaternion.conjugate(q);
    }

    //% block
    //% advanced=true
    export function multiply(q0: number[], q1: number[]): number[] {
        return Quaternion.multiply(q0, q1);
    }

    //% block
    //% advanced=true
    export function setLowPassFilterAlpha(alpha: number): void {
        famc_.setLowPassFilterAlpha(alpha);
    }

}

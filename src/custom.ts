/**
 * Custom blocks
 * icon: a Unicode identifier for an icon from the Font Awesome icon set.
 *       http://fontawesome.io/icons
 */
//% block="Custom Blocks"
//% weight=100 color=#696969 icon="\uf1b2"
namespace custom {
    const PI2 = 2 * Math.PI;

    // Quaternion
    let qw = 1.0;
    let qx = 0.0;
    let qy = 0.0;
    let qz = 0.0;
    let quaternionTimestamp = 0;

    //% block
    export function estimate(acceleration: number[], magneticForce: number[]): number[] {
        if (3 == acceleration.length) {
            famc_.setAcceleration(acceleration[0], acceleration[1], acceleration[2]);
        }
        if (3 == magneticForce.length) {
            famc_.setMagneticForce(magneticForce[0], magneticForce[1], magneticForce[2]);
        }
        famc_.estimate();
        qw = famc_.getW();
        qx = famc_.getX();
        qy = famc_.getY();
        qz = famc_.getZ();
        quaternionTimestamp++;
        return quaternion();
    }

    //% block
    export function quaternion(): number[] {
        return [qw, qx, qy, qz];
    }

    // Euler angle
    let heading = 0.0;
    let pitch = 0.0;
    let bank = 0.0;
    let azimuth = 0.0;
    let eulerTimestamp = 0;

    function convToEuler(w: number, x: number, y: number, z: number): number[] {
        // Right-Handed Coordinate System
        const ysqr = y * y;
        const t0 = 2.0 * (w * x + y * z);
        const t1 = 1.0 - 2.0 * (x * x + ysqr);
        const roll = Math.atan2(t0, t1);

        let t2 = 2.0 * (w * y - z * x);
        t2 = t2 > 1.0 ? 1.0 : (t2 < -1.0 ? -1.0 : t2);
        const pitch = Math.asin(t2);

        const t3 = 2.0 * (w * z + x * y);
        const t4 = 1.0 - 2.0 * (ysqr + z * z);
        const yaw = Math.atan2(t3, t4);

        return [yaw, pitch, roll];
    }

    function updateEulerAngle(): void {
        // Whether recalculation is needed
        if (quaternionTimestamp == eulerTimestamp) {
            return;
        }
        eulerTimestamp = quaternionTimestamp;

        // convert from Quaternion to Euler angle
        const euler = convToEuler(qw, qx, qy, qz);

        // setup Euler angle and Azimuth
        heading = euler[0];
        pitch = euler[1];
        bank = euler[2];
        if (heading > 0) {
            azimuth = PI2 - heading;
        } else {
            azimuth = -heading;
        }
    }

    //% block
    export function getAzimuthRadians(): number {
        updateEulerAngle();
        return azimuth;
    }

    //% block
    export function getAzimuth(): number {
        return Math.round(getAzimuthRadians() * 180 / Math.PI);
    }

    //% block
    export function getYawRadians(): number {
        updateEulerAngle();
        return heading;
    }

    //% block
    export function getYaw(): number {
        return Math.round(getYawRadians() * 180 / Math.PI);
    }

    //% block
    export function getPitchRadians(): number {
        updateEulerAngle();
        return pitch;
    }

    //% block
    export function getPitch(): number {
        return Math.round(getPitchRadians() * 180 / Math.PI);
    }

    //% block
    export function getRollRadians(): number {
        updateEulerAngle();
        return bank;
    }

    //% block
    export function getRoll(): number {
        return Math.round(getRollRadians() * 180 / Math.PI);
    }

}

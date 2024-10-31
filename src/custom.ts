/**
 * Custom blocks
 * icon: a Unicode identifier for an icon from the Font Awesome icon set.
 *       http://fontawesome.io/icons
 */
//% block="Custom Blocks"
//% weight=100 color=#696969 icon="\uf1b2"
namespace custom {

    //% block
    export function quaternion(): number[] {
        return [
            famc_.getW(),
            famc_.getX(),
            famc_.getY(),
            famc_.getZ(),
        ];
    }

    // Euler angle
    let eulerTimestamp = 0;
    let yaw = 0.0;
    let pitch = 0.0;
    let roll = 0.0;
    let azimuth = 0.0;
    const PI2 = 2 * Math.PI;

    function updateToEuler(): void {
        // Whether recalculation is needed
        const timestamp = famc_.getTimestamp();
        if (timestamp == eulerTimestamp) {
            return;
        }
        eulerTimestamp = timestamp;

        // Quaternion
        const w = famc_.getW();
        const x = famc_.getX();
        const y = famc_.getY();
        const z = famc_.getZ();

        // Right-Handed Coordinate System
        const ysqr = y * y;
        const t0 = 2.0 * (w * x + y * z);
        const t1 = 1.0 - 2.0 * (x * x + ysqr);
        roll = Math.atan2(t0, t1);

        let t2 = 2.0 * (w * y - z * x);
        t2 = t2 > 1.0 ? 1.0 : (t2 < -1.0 ? -1.0 : t2);
        pitch = Math.asin(t2);

        const t3 = 2.0 * (w * z + x * y);
        const t4 = 1.0 - 2.0 * (ysqr + z * z);
        yaw = Math.atan2(t3, t4);

        // Azimuth
        if (yaw > 0) {
            azimuth = PI2 - yaw;
        } else {
            azimuth = -yaw;
        }

    }

    //% block
    export function getAzimuthRadians(): number {
        updateToEuler();
        return azimuth;
    }

    //% block
    export function getAzimuth(): number {
        return Math.round(getAzimuthRadians() * 180 / Math.PI);
    }

    //% block
    export function getYawRadians(): number {
        updateToEuler();
        return yaw;
    }

    //% block
    export function getYaw(): number {
        return Math.round(getYawRadians() * 180 / Math.PI);
    }

    //% block
    export function getPitchRadians(): number {
        updateToEuler();
        return pitch;
    }

    //% block
    export function getPitch(): number {
        return Math.round(getPitchRadians() * 180 / Math.PI);
    }

    //% block
    export function getRollRadians(): number {
        updateToEuler();
        return roll;
    }

    //% block
    export function getRoll(): number {
        return Math.round(getRollRadians() * 180 / Math.PI);
    }

    //% block
    export function estimate(acceleration: number[], magneticForce: number[]) {
        famc_.setAcceleration(acceleration[0], acceleration[1], acceleration[2]);
        famc_.setMagneticForce(magneticForce[0], magneticForce[1], magneticForce[2]);
        famc_.estimateFamc();
    }
    
}

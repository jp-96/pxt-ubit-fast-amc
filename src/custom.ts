/**
 * Custom blocks
 * icon: a Unicode identifier for an icon from the Font Awesome icon set.
 *       http://fontawesome.io/icons
 */
//% block="Custom Blocks"
//% weight=100 color=#696969 icon="\uf1b2"
namespace custom {

    // Quaternion for simulator
    let estimated = 0;
    let qw = 1.0;
    let qx = 0.0;
    let qy = 0.0;
    let qz = 0.0;

    //% shim=custom::getTimestamp
    export function getTimestamp(): number {
        return estimated;
    }

    //% shim=custom::getW
    export function getW(): number {
        return qw;
    }

    //% shim=custom::getX
    export function getX(): number {
        return qx;
    }

    //% shim=custom::getY
    export function getY(): number {
        return qy;
    }

    //% shim=custom::getZ
    export function getZ(): number {
        return qz;
    }

    //% block
    export function quaternion(): number[] {
        return [
            getW(),
            getX(),
            getY(),
            getZ(),
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
        const timestamp = getTimestamp();
        if (timestamp == eulerTimestamp) {
            return;
        }
        eulerTimestamp = timestamp;

        // Quaternion
        const w = getW();
        const x = getX();
        const y = getY();
        const z = getZ();

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

    // Accelerration (normalized)
    let ax = 0.0;
    let ay = 0.0;
    let az = 1.0;

    //% shim=custom::setAcceleration
    export function setAcceleration(x: number, y: number, z: number): void {
        const norm = Math.sqrt(x * x + y * y + z * z)
        if (norm > 0) {
            ax = x / norm;
            ay = y / norm;
            az = z / norm;
        } else {
            ax = 0.0;
            ay = 0.0;
            az = 1.0;
        }
    }

    // Magnetic force (normalized)
    let mx = 1.0;
    let my = 0.0;
    let mz = 0.0;

    //% shim=custom::setMagneticForce
    export function setMagneticForce(x: number, y: number, z: number): void {
        const norm = Math.sqrt(x * x + y * y + z * z)
        if (norm > 0) {
            mx = x / norm;
            my = y / norm;
            az = z / norm;
        } else {
            mx = 1.0;
            my = 0.0;
            mz = 0.0;
        }
    }

    function estimateSimu(): void {
        qw = Math.sqrt((az + 1.0) / 2.0)
        qx = -ay / (2.0 * qw)
        qy = ax / (2.0 * qw)
        qz = 0.0
        ++estimated;
    }

    //% shim=custom::estimateFamc
    export function estimateFamc(): void {
        // for simulator
        estimateSimu();
    }

    //% block
    export function estimate(acceleration: number[], magneticForce: number[]) {
        setAcceleration(acceleration[0], acceleration[1], acceleration[2]);
        setMagneticForce(magneticForce[0], magneticForce[1], magneticForce[2]);
        estimateFamc();
    }
    
}

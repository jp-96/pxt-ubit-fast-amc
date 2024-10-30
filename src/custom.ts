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

    const REAL_100: number = 1.0;
    const REAL_070: number = Math.sin(Math.PI / 4); //0.70693004;           // sin(Math.PI / 4)
    const REAL_049: number = REAL_070 * REAL_070;   // 0.49975008
    const REAL_ZERO: number = 0.0;
    const REAL_MINUS_049: number = REAL_049 * -1;
    const REAL_MINUS_070: number = REAL_070 * -1;
    const REAL_MINUS_100: number = REAL_100 * -1

    let inv_w = REAL_100;
    let inv_x = REAL_ZERO;
    let inv_y = REAL_ZERO;
    let inv_z = REAL_ZERO;

    function updateToEuler(): void {
        // Whether recalculation is needed
        const timestamp = getTimestamp();
        if (timestamp == eulerTimestamp) {
            return;
        }
        eulerTimestamp = timestamp;

        // Quaternion
        let w = getW();
        let x = getX();
        let y = getY();
        let z = getZ();

        // Mounting rotation
        w = w * inv_w - x * inv_z - y * inv_y - z * inv_z;
        x = w * inv_x + x * inv_w + y * inv_z - z * inv_y;
        y = w * inv_y - x * inv_x + y * inv_w + z * inv_x;
        z = w * inv_z + x * inv_y - y * inv_x + z * inv_w;
        const norm = Math.sqrt(w * w + x * x + y * y + z * z);
        if (norm > 0) {
            w /= norm;
            x /= norm;
            y /= norm;
            z /= norm;
        } else {
            w = 1.0;
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }

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
    }

    //% block
    export function getYaw(): number {
        updateToEuler();
        return yaw;
    }

    //% block
    export function getPitch(): number {
        updateToEuler();
        return pitch;
    }

    //% block
    export function getRoll(): number {
        updateToEuler();
        return roll;
    }

    // Accelerration
    let ax = 0.0;
    let ay = 0.0;
    let az = 1.0;

    //% block
    //% shim=custom::setAcceleration
    export function setAcceleration(x: number, y: number, z: number): void {
        ax = x;
        ay = y;
        az = z;
    }

    // Magnetic force
    let mx = 0.0;
    let my = 0.0;
    let mz = 1.0;

    //% block
    //% shim=custom::setMagneticForce
    export function setMagneticForce(x: number, y: number, z: number): void {
        mx = x;
        my = y;
        mz = z;
    }

    //% block
    //% shim=custom::estimate
    export function estimate(): void {
        qw = 1.0;
        qx = ax;
        qy = ay;
        qz = az;
        ++estimated;
    }

    //% block
    //% shim=custom::buildErr6
    export function buildErr6(p1: number, p2: number, p3: number, p4: number, p5: number, p6: number): void {
        const a = p1 + p2 + p3 + p4 + p5 + p6;
    }

}

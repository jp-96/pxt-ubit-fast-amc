/**
 * Custom blocks
 * icon: a Unicode identifier for an icon from the Font Awesome icon set.
 *       http://fontawesome.io/icons
 */
//% block="Custom Blocks"
//% weight=100 color=#696969 icon="\uf1b2"
namespace custom {
    const PI2 = 2 * Math.PI;

    // Quaternion [w, x, y, z]
    let quaternion_ = [1.0, 0.0, 0.0, 0.0];
    let quaternionTimestamp = 0;

    //% block
    export function estimate(acceleration: number[], magneticForce: number[]): number[] {
        if ((3 == acceleration.length) && (3 == magneticForce.length)) {
            famc_.setAcceleration(acceleration[0], acceleration[1], acceleration[2]);
            famc_.setMagneticForce(magneticForce[0], magneticForce[1], magneticForce[2]);
        }
        famc_.estimate();
        const q = [famc_.getW(), famc_.getX(), famc_.getY(), famc_.getZ()];
        quaternion_ = q;
        quaternionTimestamp++;
        return q
    }

    //% block
    export function quaternion(): number[] {
        return quaternion_;
    }

    // Euler angle [heading, pitch, bank]
    let euler_ = [0.0, 0.0, 0.0]
    let eulerTimestamp = 0;

    function convToEuler(q: number[]): number[] {
        // Right-Handed Coordinate System
        if (4 != q.length) return [];
        const w = q[0];
        const x = q[1];
        const y = q[2];
        const z = q[3];

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

        // Setup Euler angle
        euler_ = convToEuler(quaternion());
    }

    //% block
    export function getAzimuthRadians(): number {
        updateEulerAngle();
        const heading = getHeadingRadians();
        if (heading > 0) {
            return PI2 - heading;
        } else {
            return -1.0 * heading;
        }
    }

    //% block
    export function getAzimuth(): number {
        return Math.round(getAzimuthRadians() * 180 / Math.PI);
    }

    //% block
    export function getHeadingRadians(): number {
        updateEulerAngle();
        return euler_[0];
    }

    //% block
    export function getHeading(): number {
        return Math.round(getHeadingRadians() * 180 / Math.PI);
    }

    //% block
    export function getPitchRadians(): number {
        updateEulerAngle();
        return euler_[1];
    }

    //% block
    export function getPitch(): number {
        return Math.round(getPitchRadians() * 180 / Math.PI);
    }

    //% block
    export function getBankRadians(): number {
        updateEulerAngle();
        return euler_[2];
    }

    //% block
    export function getBank(): number {
        return Math.round(getBankRadians() * 180 / Math.PI);
    }

}

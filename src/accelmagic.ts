enum AngleRpy {
    Roll,
    Pitch,
    Yaw,
    Azimuth,
}

/**
 * A Simplified Analytic Attitude Determination Algorithm
 * Using Accelerometer and Magnetometer on micro:bit.
 * 
 */
//% block="Accel Magic"
//% weight=100 color=#696969 icon="\uf1b2"
namespace accelmagic {

    //% block
    //% advanced=true
    export function setLowPassFilterAlpha(alpha: number): void {
        famc_.setLowPassFilterAlpha(alpha);
    }

    //% block
    export function updateAcceleration(x: number, y: number, z: number): void {
        famc_.updateAcceleration(x, y, z);
    }

    //% block
    export function updateMagneticForce(x: number, y: number, z: number): void {
        famc_.updateMagneticForce(x, y, z);
    }

    //% block
    export function estimate(): Quaternion {
        famc_.estimate();
        return new Quaternion(famc_.getW(), famc_.getX(), famc_.getY(), famc_.getZ());
    }

    //% block
    //% advanced=true
    export function createQuat(w: number, x: number, y: number, z: number): Quaternion {
        return (new Quaternion(w, x, y, z)).normalize();
    }

    //% block
    //% advanced=true
    export function normalizeQuat(q: Quaternion): Quaternion {
        return q.normalize();
    }

    //% block
    //% advanced=true
    export function conjugateQuat(q: Quaternion): Quaternion {
        return q.conjugate();
    }

    //% block
    //% advanced=true
    export function multiplyQuats(q0: Quaternion, q1: Quaternion): Quaternion {
        return q0.multiply(q1);
    }

    //% block
    //% advanced=true
    export function quatToArray(q: Quaternion): number[] {
        return q.toArray();
    }

    //% block
    export function quatToRpy(q: Quaternion): EulerAngles {
        return EulerAngles.fromQuaternion(q);
    }

    //% block
    export function getEulerAngles(eulerAngles: EulerAngles, angleRpy: AngleRpy): number {
        switch (angleRpy){
            case AngleRpy.Roll:
                return eulerAngles.roll;
            case AngleRpy.Pitch:
                return eulerAngles.pitch;
            case AngleRpy.Yaw:
                return eulerAngles.yaw;
            case AngleRpy.Azimuth:
                return eulerAngles.getAzimuth();
            default:
                return 0;
        }
    }

    export class Quaternion {

        public static identity = new Quaternion(1.0, 0.0, 0.0, 0.0);

        /**
         * build from array.
         * @param q [w, x, y, z]
         * @returns instance of Quaternion
         */
        public static fromArray(q: number[]): Quaternion {
            if (4 == q.length) {
                return new Quaternion(q[0], q[1], q[2], q[3]);
            } else {
                return Quaternion.identity;
            }
        }

        /**
         * constructor
         * @param w Scalar part (w): Represents the magnitude or angle of rotation.
         * @param x Vector part (x): Indicates the x-axis of rotation.
         * @param y Vector part (y): Indicates the y-axis of rotation.
         * @param z Vector part (z): Indicates the z-axis of rotation.
         */
        constructor(public w: number, public x: number, public y: number, public z: number) {
            //
        }

        public normalize(): Quaternion {
            let w = this.w;
            let x = this.x;
            let y = this.y;
            let z = this.z;

            const norm = Math.sqrt(w * w + x * x + y * y + z * z);
            if (0.0 < norm) {
                // Normalize it
                const oneOverNorm = 1.0 / norm;
                w *= oneOverNorm;
                x *= oneOverNorm;
                y *= oneOverNorm;
                z *= oneOverNorm;
                return new Quaternion(w, x, y, z);
            } else {
                return Quaternion.identity;
            }
        }

        public conjugate(): Quaternion {
            return new Quaternion(this.w, -this.x, -this.y, -this.z);
        }

        public multiply(q: Quaternion): Quaternion {
            return new Quaternion(
                this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z,
                this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
                this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x,
                this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w
            );
        }

        /**
         * to array.
         * @returns [w, x, y, z]
         */
        public toArray(): number[] {
            return [this.w, this.x, this.y, this.z];
        }

    }

    export class EulerAngles {

        private static PI_2 = Math.PI * 2.0;

        public static identity = new EulerAngles(0.0, 0.0, 0.0);

        /**
         * build from array.
         * @param rpy [roll, pitch, yaw]
         * @returns instance of EulerAngles
         */
        public static fromArray(rpy: number[]): EulerAngles {
            if (3 == rpy.length) {
                return new EulerAngles(rpy[0], rpy[1], rpy[2]);
            } else {
                return EulerAngles.identity;
            }
        }

        /**
         * build from Quaternion.
         * @param q instance of Quaternion
         * @returns instance of EulerAngles
         */
        public static fromQuaternion(q: Quaternion): EulerAngles {

            const ysqr = q.y * q.y;
            const t0 = 2.0 * (q.w * q.x + q.y * q.z);
            const t1 = 1.0 - 2.0 * (q.x * q.x + ysqr);
            const roll = Math.atan2(t0, t1);
    
            let t2 = 2.0 * (q.w * q.y - q.z * q.x);
            t2 = t2 > 1.0 ? 1.0 : (t2 < -1.0 ? -1.0 : t2);
            const pitch = Math.asin(t2);
    
            const t3 = 2.0 * (q.w * q.z + q.x * q.y);
            const t4 = 1.0 - 2.0 * (ysqr + q.z * q.z);
            const yaw = Math.atan2(t3, t4);
    
            return new EulerAngles(roll, pitch, yaw);
        }

        /**
         * constructor
         * @param roll Rotation around the X-axis. It’s how much tilts to its sides.
         * @param pitch Rotation around the Y-axis. It’s how much nose is up or down.
         * @param yaw Rotation around the Z-axis.
         */
        constructor(public roll: number, public pitch: number, public yaw: number) {
            //
        }

        public getAzimuth(): number {
            if (this.yaw > 0) {
                return EulerAngles.PI_2 - this.yaw;
            } else {
                return -1.0 * this.yaw;
            }
        }

        /**
         * to array.
         * @returns [roll, pitch, yaw]
         */
        public toArray(): number[] {
            return [this.roll, this.pitch, this.yaw];
        }

    }

    const C180_OVER_PI = 180 / Math.PI;
    const CPI_OVER_180 = Math.PI / 180;

    //% block
    export function toIntegerDegree(radian: number): number {
        return Math.round(toDegree(radian));
    }

    //% block
    //% advanced=true
    export function toDegree(radian: number): number {
        return radian * C180_OVER_PI;
    }

    //% block
    //% advanced=true
    export function toRadian(degree: number): number {
        return degree * CPI_OVER_180;
    }

}

enum AngleRPY {
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
//% groups="['Quaternion', 'EulerAngles', 'Sensor']"
namespace accelmagic {

    //% block
    //% group="Quaternion"
    //% weight=270
    export function quat(w: number, x: number, y: number, z: number): Quaternion {
        return new Quaternion(w, x, y, z)
    }

    //% block
    //% group="Quaternion"
    //% weight=210
    //% advanced=true
    export function quatFrom(q: number[]): Quaternion {
        return Quaternion.fromArray(q);
    }

    //% block
    //% group="Quaternion"
    //% weight=260
    export function normalize(q: Quaternion): Quaternion {
        return q.normalize();
    }

    //% block
    //% group="Quaternion"
    //% weight=250
    export function conjugate(q: Quaternion): Quaternion {
        return q.conjugate();
    }

    //% block
    //% group="Quaternion"
    //% weight=240
    export function multiply(a: Quaternion, b: Quaternion): Quaternion {
        return a.multiply(b);
    }

    //% block
    //% group="Quaternion"
    //% weight=230
    export function diff(a: Quaternion, b: Quaternion): Quaternion {
        return a.conjugate().multiply(b);
    }

    //% block
    //% group="Quaternion"
    //% weight=220
    export function quatFromRpy(rpy: EulerAngles): Quaternion {
        return Quaternion.fromEulerAngles(rpy);
    }

    //% block
    //% group="Quaternion"
    //% weight=210
    //% advanced=true
    export function quatAsArray(q: Quaternion): number[] {
        return q.toArray();
    }

    //% block
    //% group="EulerAngles"
    //% weight=200
    export function rpy(roll: number, pitch: number, yaw: number): EulerAngles {
        return new EulerAngles(roll, pitch, yaw);
    }

    //% block
    //% group="EulerAngles"
    //% weight=180
    export function rpyFrom(rpy: number[]): EulerAngles {
        return EulerAngles.fromArray(rpy);
    }
    //% block
    //% group="EulerAngles"
    //% weight=190
    export function angle(rpy: EulerAngles, angleRPY: AngleRPY): number {
        switch (angleRPY) {
            case AngleRPY.Roll:
                return rpy.roll;
            case AngleRPY.Pitch:
                return rpy.pitch;
            case AngleRPY.Yaw:
                return rpy.yaw;
            case AngleRPY.Azimuth:
                return rpy.getAzimuth();
            default:
                return 0;
        }
    }

    //% block
    //% group="EulerAngles"
    //% weight=180
    export function rpyFromQuat(q: Quaternion): EulerAngles {
        return EulerAngles.fromQuaternion(q);
    }

    //% block
    //% group="EulerAngles"
    //% weight=170
    //% advanced=true
    export function rpyAsArray(rpy: EulerAngles): number[] {
        return rpy.toArray();
    }

    const C180_OVER_PI = 180 / Math.PI;
    const CPI_OVER_180 = Math.PI / 180;

    //% block
    //% group="EulerAngles"
    //% weight=160
    //% advanced=true
    export function intDeg(radian: number): number {
        return Math.round(decDeg(radian));
    }

    //% block
    //% group="EulerAngles"
    //% weight=150
    //% advanced=true
    export function decDeg(radian: number): number {
        return radian * C180_OVER_PI;
    }

    //% block
    //% group="EulerAngles"
    //% weight=140
    //% advanced=true
    export function rad(degree: number): number {
        return degree * CPI_OVER_180;
    }

    export class Quaternion {

        public static identity = new Quaternion(1.0, 0.0, 0.0, 0.0);

        /**
         * build from array.
         * @param q [w, x, y, z]
         * @returns instance of Quaternion. If there is an inconsistency, it returns the identity.
         */
        public static fromArray(q: number[]): Quaternion {
            if (4 == q.length) {
                return new Quaternion(q[0], q[1], q[2], q[3]);
            } else {
                return Quaternion.identity;
            }
        }

        /**
         * build from Euler angles.
         * @param rpy instance of EulerAngles
         * @returns instance of Quaternion
         */
        public static fromEulerAngles(rpy: EulerAngles): Quaternion {
            const roll = rpy.roll;
            const pitch = rpy.pitch;
            const yaw = rpy.yaw;

            const cy = Math.cos(yaw * 0.5);
            const sy = Math.sin(yaw * 0.5);
            const cp = Math.cos(pitch * 0.5);
            const sp = Math.sin(pitch * 0.5);
            const cr = Math.cos(roll * 0.5);
            const sr = Math.sin(roll * 0.5);

            const w = cr * cp * cy + sr * sp * sy;
            const x = sr * cp * cy - cr * sp * sy;
            const y = cr * sp * cy + sr * cp * sy;
            const z = cr * cp * sy - sr * sp * cy;

            return new Quaternion(w, x, y, z);
        }

        /**
         * constructor
         * @param w Scalar part (w): Represents the magnitude or angle of rotation.
         * @param x Vector part (x): Indicates the x-axis of rotation.
         * @param y Vector part (y): Indicates the y-axis of rotation.
         * @param z Vector part (z): Indicates the z-axis of rotation.
         */
        constructor(public w: number, public x: number, public y: number, public z: number) {
            //this.normalize();
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
            // return new Quaternion(this.w, -this.x, -this.y, -this.z);
            return new Quaternion(-this.w, this.x, this.y, this.z);
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
         * @returns instance of EulerAngles. If there is an inconsistency, it returns the identity.
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

}

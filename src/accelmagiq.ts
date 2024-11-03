/**
 * Enum for selecting angles in Euler angles.
 */
enum AngleRPY {
    /** Roll angle (rotation around the X-axis) */
    //% block="Roll"
    Roll,
    /** Pitch angle (rotation around the Y-axis) */
    //% block="Pitch"
    Pitch,
    /** Yaw angle (rotation around the Z-axis) */
    //% block="Yaw"
    Yaw,
    /** Azimuth angle (derived from the yaw angle) */
    //% block="Azimuth"
    Azimuth,
}

/**
 * A Simplified Analytic Attitude Determination Algorithm
 * using Accelerometer and Magnetometer on micro:bit.
 * 
 * It's like magic! This algorithm turns raw data into accurate and efficient quaternion estimations,
 * transforming your projects and making you go, "Wow!" Using the handy micro:bit, it feels like trying
 * out quaternions is pure magic. AccelMagiQ brings a touch of enchantment to the technical world.
 * We hope this helps you in your learning journey and sparks your curiosity about the fascinating
 * world of quaternions.
 * 
 * "AccelMagiQ" refers to this amazing algorithm that combines accelerometer and magnetometer data to
 * create precise quaternion calculations. It's a playful blend of 'acceleration', 'magnetometer', and 'magic' with
 * quaternions ('Q'), making advanced concepts seem like magic!
 */
//% block="AccelMagiQ"
//% weight=100 color=#696969 icon="\uf1b2"
//% groups="['Quaternion', 'EulerAngles', 'Sensor']"
namespace accelmagiq {

    /**
     * Creates a quaternion from an array.
     * @param q An array [w, x, y, z].
     * @returns An instance of Quaternion. If there is an inconsistency, it returns the identity quaternion.
     */
    //% block="Create Quaternion from Array %q"
    //% group="Quaternion"
    //% weight=138
    export function quatFrom(q: number[]): Quaternion {
        return Quaternion.fromArray(q);
    }

    /**
     * Normalizes the quaternion.
     * @param q An instance of Quaternion.
     * @returns A normalized Quaternion.
     */
    //% block="Normalize Quaternion %q"
    //% group="Quaternion"
    //% weight=137
    export function normalize(q: Quaternion): Quaternion {
        return q.normalize();
    }

    /**
     * Computes the conjugate of the quaternion.
     * @param q An instance of Quaternion.
     * @returns The conjugate of the quaternion.
     */
    //% block="Conjugate Quaternion %q"
    //% group="Quaternion"
    //% weight=136
    export function conjugate(q: Quaternion): Quaternion {
        return q.conjugate();
    }

    /**
     * Multiplies two quaternions.
     * @param a The first quaternion.
     * @param b The second quaternion.
     * @returns The product of the two quaternions.
     */
    //% block="Multiply Quaternions %a and %b"
    //% group="Quaternion"
    //% weight=135
    export function multiply(a: Quaternion, b: Quaternion): Quaternion {
        return a.multiply(b);
    }

    /**
     * Computes the difference between two quaternions.
     * @param a The first quaternion.
     * @param b The second quaternion.
     * @returns The difference between the two quaternions.
     */
    //% block="Difference of Quaternions %a and %b"
    //% group="Quaternion"
    //% weight=134
    export function diff(a: Quaternion, b: Quaternion): Quaternion {
        return a.conjugate().multiply(b);
    }

    /**
     * Computes the rotation angle of the quaternion.
     * @param q An instance of Quaternion.
     * @returns The rotation angle in radians.
     */
    //% block="Quaternion Rotation Angle %q"
    //% group="Quaternion"
    //% weight=133
    export function quatRotationAngle(q: Quaternion): number {
        return q.getRotationAngle();
    }

    /**
     * Creates a quaternion.
     * @param w Scalar part (w): Represents the magnitude or angle of rotation.
     * @param x Vector part (x): Indicates the x-axis of rotation.
     * @param y Vector part (y): Indicates the y-axis of rotation.
     * @param z Vector part (z): Indicates the z-axis of rotation.
     * @returns An instance of Quaternion.
     */
    //% block="Create Quaternion w %w x %x y %y z %z"
    //% inlineInputMode=inline
    //% group="Quaternion"
    //% weight=132
    //% advanced=true
    export function quat(w: number, x: number, y: number, z: number): Quaternion {
        return new Quaternion(w, x, y, z)
    }

    /**
     * Creates a quaternion from Euler angles.
     * @param rpy An instance of EulerAngles.
     * @returns An instance of Quaternion.
     */
    //% block="Create Quaternion from Euler Angles %rpy"
    //% group="Quaternion"
    //% weight=131
    //% advanced=true
    export function quatFromRpy(rpy: EulerAngles): Quaternion {
        return Quaternion.fromEulerAngles(rpy);
    }

    /**
     * Converts the quaternion to an array.
     * @param q An instance of Quaternion.
     * @returns An array [w, x, y, z] representing the quaternion.
     */
    //% block="Quaternion to Array %q"
    //% group="Quaternion"
    //% weight=130
    //% advanced=true
    export function quatAsArray(q: Quaternion): number[] {
        return q.toArray();
    }

    /**
     * Creates an instance of EulerAngles from a quaternion.
     * @param q An instance of Quaternion.
     * @returns An instance of EulerAngles.
     */
    //% block="Create Euler Angles from Quaternion %q"
    //% group="EulerAngles"
    //% weight=124
    export function rpyFromQuat(q: Quaternion): EulerAngles {
        return EulerAngles.fromQuaternion(q);
    }

    /**
     * Retrieves a specific angle from EulerAngles.
     * @param rpy An instance of EulerAngles.
     * @param angleRPY The angle to retrieve (Roll, Pitch, Yaw, or Azimuth).
     * @returns The specified angle in radians.
     */
    //% block="Get Angle from Euler Angles %rpy angle %angleRPY"
    //% group="EulerAngles"
    //% weight=123
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

    /**
     * Creates an instance of EulerAngles.
     * @param roll Rotation around the X-axis. It’s how much tilts to its sides.
     * @param pitch Rotation around the Y-axis. It’s how much nose is up or down.
     * @param yaw Rotation around the Z-axis.
     * @returns An instance of EulerAngles.
     */
    //% block="Create Euler Angles roll %roll pitch %pitch yaw %yaw"
    //% group="EulerAngles"
    //% weight=122
    //% advanced=true
    export function rpy(roll: number, pitch: number, yaw: number): EulerAngles {
        return new EulerAngles(roll, pitch, yaw);
    }

    /**
     * Creates an instance of EulerAngles from an array.
     * @param rpy An array [roll, pitch, yaw].
     * @returns An instance of EulerAngles. If there is an inconsistency, it returns the identity.
     */
    //% block="Create Euler Angles from Array %rpy"
    //% group="EulerAngles"
    //% weight=121
    //% advanced=true
    export function rpyFrom(rpy: number[]): EulerAngles {
        return EulerAngles.fromArray(rpy);
    }

    /**
     * Converts the Euler angles to an array.
     * @param rpy An instance of EulerAngles.
     * @returns An array [roll, pitch, yaw] representing the Euler angles in radians.
     */
    //% block="Euler Angles to Array %rpy"
    //% group="EulerAngles"
    //% weight=120
    //% advanced=true
    export function rpyAsArray(rpy: EulerAngles): number[] {
        return rpy.toArray();
    }

    const C180_OVER_PI = 180 / Math.PI;
    const CPI_OVER_180 = Math.PI / 180;

    /**
     * Converts radians to integer degrees.
     * @param radian The angle in radians.
     * @returns The angle in integer degrees.
     */
    //% block="Radians %radian to Integer Degrees"
    //% group="EulerAngles"
    //% weight=112
    export function intDeg(radian: number): number {
        return Math.round(decDeg(radian));
    }

    /**
     * Converts radians to decimal degrees.
     * @param radian The angle in radians.
     * @returns The angle in decimal degrees.
     */
    //% block="Radians %radian to Decimal Degrees"
    //% group="EulerAngles"
    //% weight=111
    //% advanced=true
    export function decDeg(radian: number): number {
        return radian * C180_OVER_PI;
    }

    /**
     * Converts degrees to radians.
     * @param degree The angle in degrees.
     * @returns The angle in radians.
     */
    //% block="Degrees %degree to Radians"
    //% group="EulerAngles"
    //% weight=110
    //% advanced=true
    export function rad(degree: number): number {
        return degree * CPI_OVER_180;
    }

    /**
     * A class to represent a quaternion for 3D rotations.
     */
    export class Quaternion {

        /**
         * The identity quaternion.
         */
        private static identity = new Quaternion(1.0, 0.0, 0.0, 0.0);

        /**
         * Creates a quaternion from an array.
         * @param q [w, x, y, z] - The quaternion components.
         * @returns An instance of Quaternion. If there is an inconsistency, it returns the identity quaternion.
         */
        public static fromArray(q: number[]): Quaternion {
            if (4 == q.length) {
                return new Quaternion(q[0], q[1], q[2], q[3]);
            } else {
                return Quaternion.identity;
            }
        }

        /**
         * Creates a quaternion from Euler angles.
         * @param rpy An instance of EulerAngles.
         * @returns An instance of Quaternion.
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
         * Constructor for creating a quaternion.
         * @param w Scalar part (w): Represents the magnitude or angle of rotation.
         * @param x Vector part (x): Indicates the x-axis of rotation.
         * @param y Vector part (y): Indicates the y-axis of rotation.
         * @param z Vector part (z): Indicates the z-axis of rotation.
         */
        constructor(public w: number, public x: number, public y: number, public z: number) {
            // this.normalize();
        }

        /**
         * Normalizes the quaternion.
         * @returns A unit quaternion or the identity quaternion if normalization fails.
         */
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

        /**
         * Computes the conjugate of the quaternion.
         * @returns The conjugate of the quaternion.
         */
        public conjugate(): Quaternion {
            return new Quaternion(this.w, -this.x, -this.y, -this.z);
        }

        /**
         * Multiplies this quaternion by another quaternion.
         * @param q Another quaternion.
         * @returns The product of the two quaternions.
         */
        public multiply(q: Quaternion): Quaternion {
            return new Quaternion(
                this.w * q.w - this.x * q.x - this.y * q.y - this.z * q.z,
                this.w * q.x + this.x * q.w + this.y * q.z - this.z * q.y,
                this.w * q.y - this.x * q.z + this.y * q.w + this.z * q.x,
                this.w * q.z + this.x * q.y - this.y * q.x + this.z * q.w
            );
        }

        /**
         * Converts the quaternion to an array.
         * @returns An array [w, x, y, z] representing the quaternion.
         */
        public toArray(): number[] {
            return [this.w, this.x, this.y, this.z];
        }

        /**
         * Computes the rotation angle of the quaternion.
         * @returns The rotation angle in radians.
         */
        public getRotationAngle(): number {
            const q = this.normalize();
            return 2.0 * Math.acos(q.w);
        }

    }

    /**
     * A class to represent Euler angles for 3D rotations.
     */
    export class EulerAngles {

        /**
         * The value of 2xPI (full circle in radians).
         */
        private static PI_2 = Math.PI * 2.0;

        /**
         * The identity Euler angles (0, 0, 0).
         */
        private static identity = new EulerAngles(0.0, 0.0, 0.0);

        /**
         * Creates an instance of EulerAngles from an array.
         * @param rpy [roll, pitch, yaw] - The Euler angles in radians.
         * @returns An instance of EulerAngles. If there is an inconsistency, it returns the identity.
         */
        public static fromArray(rpy: number[]): EulerAngles {
            if (3 == rpy.length) {
                return new EulerAngles(rpy[0], rpy[1], rpy[2]);
            } else {
                return EulerAngles.identity;
            }
        }

        /**
         * Creates an instance of EulerAngles from a Quaternion.
         * @param q An instance of Quaternion.
         * @returns An instance of EulerAngles.
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
         * Constructor for creating an instance of EulerAngles.
         * @param roll Rotation around the X-axis. It’s how much tilts to its sides.
         * @param pitch Rotation around the Y-axis. It’s how much nose is up or down.
         * @param yaw Rotation around the Z-axis.
         */
        constructor(public roll: number, public pitch: number, public yaw: number) {
            //
        }

        /**
         * Computes the azimuth angle from the yaw angle.
         * @returns The azimuth angle in radians.
         */
        public getAzimuth(): number {
            if (this.yaw > 0) {
                return EulerAngles.PI_2 - this.yaw;
            } else {
                return -1.0 * this.yaw;
            }
        }

        /**
         * Converts the Euler angles to an array.
         * @returns An array [roll, pitch, yaw] representing the Euler angles in radians.
         */
        public toArray(): number[] {
            return [this.roll, this.pitch, this.yaw];
        }

    }

}

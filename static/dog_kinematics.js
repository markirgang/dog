/**
 * Dog Robot 17-DOF Kinematics & Gait Engine
 *
 * Joint layout:
 *   0  head_pan          1  head_tilt
 *   2  tail_wag          3  l_ear            4  r_ear
 *   5  lf_hip_yaw        6  lf_hip_pitch     7  lf_knee
 *   8  rf_hip_yaw        9  rf_hip_pitch    10  rf_knee
 *  11  lr_hip_yaw       12  lr_hip_pitch    13  lr_knee
 *  14  rr_hip_yaw       15  rr_hip_pitch    16  rr_knee
 *
 * 5 body/head joints + 4 legs x 3 joints = 17 servos.
 */

class DogKinematics {
    constructor() {
        this.JOINTS = [
            'head_pan', 'head_tilt', 'tail_wag', 'l_ear', 'r_ear',
            'lf_hip_yaw', 'lf_hip_pitch', 'lf_knee',
            'rf_hip_yaw', 'rf_hip_pitch', 'rf_knee',
            'lr_hip_yaw', 'lr_hip_pitch', 'lr_knee',
            'rr_hip_yaw', 'rr_hip_pitch', 'rr_knee'
        ];

        this.HOME = {
            head_pan: 0, head_tilt: 0, tail_wag: 0, l_ear: 0, r_ear: 0,
            lf_hip_yaw: 0, lf_hip_pitch: 0, lf_knee: 5,
            rf_hip_yaw: 0, rf_hip_pitch: 0, rf_knee: 5,
            lr_hip_yaw: 0, lr_hip_pitch: 0, lr_knee: 5,
            rr_hip_yaw: 0, rr_hip_pitch: 0, rr_knee: 5
        };

        this.LIMITS = {
            head_pan: [-80, 80], head_tilt: [-40, 40], tail_wag: [-70, 70],
            l_ear: [-40, 40], r_ear: [-40, 40],
            lf_hip_yaw: [-45, 45], lf_hip_pitch: [-90, 90], lf_knee: [-120, 5],
            rf_hip_yaw: [-45, 45], rf_hip_pitch: [-90, 90], rf_knee: [-120, 5],
            lr_hip_yaw: [-45, 45], lr_hip_pitch: [-90, 90], lr_knee: [-120, 5],
            rr_hip_yaw: [-45, 45], rr_hip_pitch: [-90, 90], rr_knee: [-120, 5]
        };

        this.LEGS = ['lf', 'rf', 'lr', 'rr'];
        this.THIGH_LEN = 100;
        this.SHIN_LEN = 100;
        this.DEFAULT_HEIGHT = 210;

        this.calibrationTrims = {};
        this.JOINTS.forEach(name => { this.calibrationTrims[name] = 0; });

        this.activePose = 'Stand';
        this.walkPhase = 0;
        this.stepHeight = 35;
        this.stepLength = 60;
        this.gaitType = 'trot';
    }

    setCalibration(trims) {
        this.JOINTS.forEach(name => {
            this.calibrationTrims[name] = trims && typeof trims[name] === 'number' ? trims[name] : 0;
        });
    }

    clamp(name, value) {
        const lim = this.LIMITS[name] || [-180, 180];
        return Math.max(lim[0], Math.min(lim[1], value));
    }

    basePose() {
        return Object.assign({}, this.HOME);
    }

    applyOverrides(joints, overrides) {
        if (!overrides) return;
        Object.keys(overrides).forEach(key => {
            if (joints[key] !== undefined) joints[key] = overrides[key];
        });
    }

    solveLegIK(target, leg) {
        const x = target.x || 0;
        const y = target.y || -this.DEFAULT_HEIGHT;
        const z = target.z || 0;

        const hipYaw = Math.atan2(x, z);
        const planar = Math.sqrt(x * x + z * z);
        let d = Math.sqrt(planar * planar + y * y);
        const maxReach = this.THIGH_LEN + this.SHIN_LEN - 1;
        const minReach = Math.abs(this.THIGH_LEN - this.SHIN_LEN) + 1;
        d = Math.max(minReach, Math.min(maxReach, d));

        const cosKnee = (this.THIGH_LEN * this.THIGH_LEN + this.SHIN_LEN * this.SHIN_LEN - d * d) /
            (2 * this.THIGH_LEN * this.SHIN_LEN);
        const knee = -(Math.PI - Math.acos(Math.max(-1, Math.min(1, cosKnee))));

        const cosHip = (this.THIGH_LEN * this.THIGH_LEN + d * d - this.SHIN_LEN * this.SHIN_LEN) /
            (2 * this.THIGH_LEN * d);
        const hipOffset = Math.acos(Math.max(-1, Math.min(1, cosHip)));
        const hipPitch = Math.atan2(planar, -y) - hipOffset;

        const prefix = leg + '_';
        return {
            [prefix + 'hip_yaw']: this.clamp(prefix + 'hip_yaw', hipYaw * 180 / Math.PI),
            [prefix + 'hip_pitch']: this.clamp(prefix + 'hip_pitch', hipPitch * 180 / Math.PI),
            [prefix + 'knee']: this.clamp(prefix + 'knee', knee * 180 / Math.PI)
        };
    }

    poseFromName(name) {
        const P = {
            'Stand': {},
            'Sit': {
                lr_hip_pitch: -55, rr_hip_pitch: -55, lr_knee: 95, rr_knee: 95,
                lf_hip_pitch: 8, rf_hip_pitch: 8, lf_knee: 0, rf_knee: 0,
                tail_wag: 10
            },
            'Lie Down / Sleep': {
                lf_hip_pitch: -35, rf_hip_pitch: -35, lr_hip_pitch: -35, rr_hip_pitch: -35,
                lf_knee: 100, rf_knee: 100, lr_knee: 100, rr_knee: 100,
                head_tilt: 15, tail_wag: -15
            },
            'Wag Tail': { tail_wag: 30, head_tilt: 5 },
            'Wave Right Paw': {
                rf_hip_pitch: -75, rf_knee: 65, rf_hip_yaw: 18,
                head_pan: 10, head_tilt: 6, tail_wag: 20
            },
            'Wave Left Paw': {
                lf_hip_pitch: -75, lf_knee: 65, lf_hip_yaw: -18,
                head_pan: -10, head_tilt: 6, tail_wag: -20
            },
            'Shake Hands': {
                rf_hip_pitch: -70, rf_knee: 80, rf_hip_yaw: 10,
                head_tilt: -6, tail_wag: 15
            },
            'Beg': {
                lr_hip_pitch: -60, rr_hip_pitch: -60, lr_knee: 95, rr_knee: 95,
                lf_hip_pitch: -80, rf_hip_pitch: -80, lf_knee: 70, rf_knee: 70,
                head_tilt: -10, tail_wag: 10
            },
            'Bow / Play': {
                lf_hip_pitch: 22, rf_hip_pitch: 22, lf_knee: 0, rf_knee: 0,
                lr_hip_pitch: -28, rr_hip_pitch: -28, lr_knee: 38, rr_knee: 38,
                head_tilt: -18, tail_wag: 25
            },
            'Dance': {
                lf_hip_pitch: -8, rf_hip_pitch: 8, lr_hip_pitch: 8, rr_hip_pitch: -8,
                lf_knee: 12, rf_knee: 12, lr_knee: 12, rr_knee: 12,
                tail_wag: 30, head_tilt: 8
            },
            'Walk in Place': {},
            'Run in Place': {},
            'Attack / Guard': {
                lf_hip_pitch: -18, rf_hip_pitch: -18, lr_hip_pitch: -22, rr_hip_pitch: -22,
                lf_knee: 35, rf_knee: 35, lr_knee: 40, rr_knee: 40,
                head_tilt: -15, tail_wag: -10
            },
            'Curious Tilt': { head_pan: 15, head_tilt: 22, l_ear: -18, r_ear: 10, tail_wag: 8 },
            'Greeting': { head_tilt: 8, tail_wag: 30, l_ear: -8, r_ear: 8 },
            'Stretch': {
                lf_hip_pitch: 20, rf_hip_pitch: 20, lf_knee: 0, rf_knee: 0,
                lr_hip_pitch: -30, rr_hip_pitch: -30, lr_knee: 45, rr_knee: 45,
                head_tilt: -12
            },
            'Low Stance': {
                lf_hip_pitch: -25, rf_hip_pitch: -25, lr_hip_pitch: -25, rr_hip_pitch: -25,
                lf_knee: 50, rf_knee: 50, lr_knee: 50, rr_knee: 50
            },
            'High Stance': {
                lf_hip_pitch: 8, rf_hip_pitch: 8, lr_hip_pitch: 8, rr_hip_pitch: 8,
                lf_knee: 0, rf_knee: 0, lr_knee: 0, rr_knee: 0
            },
            'Roll Over': { tail_wag: 20, l_ear: -15, r_ear: 15 },
            'Shake Off': { l_ear: -20, r_ear: 20, tail_wag: 15 }
        };
        return P[name] || {};
    }

    bodyFromName(name) {
        const B = {
            'Stand': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'Sit': { height: -55, roll: 0, pitch: 22, yaw: 0 },
            'Lie Down / Sleep': { height: -150, roll: 0, pitch: 0, yaw: 0 },
            'Wag Tail': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'Wave Right Paw': { height: 0, roll: -4, pitch: 0, yaw: 0 },
            'Wave Left Paw': { height: 0, roll: 4, pitch: 0, yaw: 0 },
            'Shake Hands': { height: 0, roll: -5, pitch: -4, yaw: 0 },
            'Beg': { height: -40, roll: 0, pitch: -28, yaw: 0 },
            'Bow / Play': { height: -30, roll: 0, pitch: 22, yaw: 0 },
            'Dance': { height: 8, roll: 8, pitch: 0, yaw: 12 },
            'Walk in Place': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'Run in Place': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'Attack / Guard': { height: -25, roll: 0, pitch: 10, yaw: 0 },
            'Curious Tilt': { height: 0, roll: 6, pitch: 0, yaw: 0 },
            'Greeting': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'Stretch': { height: -25, roll: 0, pitch: 20, yaw: 0 },
            'Low Stance': { height: -45, roll: 0, pitch: 0, yaw: 0 },
            'High Stance': { height: 25, roll: 0, pitch: 0, yaw: 0 },
            'Roll Over': { height: 0, roll: 0, pitch: 0, yaw: 0 },
            'Shake Off': { height: 0, roll: 0, pitch: 0, yaw: 0 }
        };
        return Object.assign({ height: 0, roll: 0, pitch: 0, yaw: 0 }, B[name] || {});
    }

    computePose(name, timeMs) {
        this.activePose = name || 'Stand';
        const t = (timeMs === undefined ? performance.now() : timeMs);
        const joints = this.basePose();
        this.applyOverrides(joints, this.poseFromName(this.activePose));
        const body = this.bodyFromName(this.activePose);

        if (this.activePose === 'Walk in Place') {
            return this.computeGait(1 / 30, { fwd: 1, strafe: 0, turn: 0 }, 'walk', body);
        }
        if (this.activePose === 'Run in Place') {
            return this.computeGait(1 / 24, { fwd: 1, strafe: 0, turn: 0 }, 'run', body);
        }

        if (this.activePose === 'Wag Tail' || this.activePose === 'Greeting') {
            joints.tail_wag = Math.sin(t * 0.018) * 45;
            joints.head_tilt += Math.sin(t * 0.004) * 4;
        } else if (this.activePose === 'Wave Right Paw' || this.activePose === 'Shake Hands') {
            joints.rf_hip_pitch += Math.sin(t * 0.012) * 18;
            joints.rf_knee += Math.sin(t * 0.012) * 12;
            joints.tail_wag = Math.sin(t * 0.016) * 25;
        } else if (this.activePose === 'Wave Left Paw') {
            joints.lf_hip_pitch += Math.sin(t * 0.012) * 18;
            joints.lf_knee += Math.sin(t * 0.012) * 12;
            joints.tail_wag = Math.sin(t * 0.016 + Math.PI) * 25;
        } else if (this.activePose === 'Dance') {
            const s = Math.sin(t * 0.004);
            const c = Math.cos(t * 0.0032);
            joints.tail_wag = 35 * s;
            joints.head_pan = 18 * c;
            joints.head_tilt = 8 * s;
            joints.l_ear = -15 * Math.abs(s);
            joints.r_ear = 15 * Math.abs(s);
            joints.lf_knee = 12 + 14 * Math.max(0, s);
            joints.rf_knee = 12 + 14 * Math.max(0, -s);
            joints.lr_knee = 12 + 14 * Math.max(0, -s);
            joints.rr_knee = 12 + 14 * Math.max(0, s);
            body.roll = 10 * s;
            body.yaw = 12 * s;
            body.height = 8 + 6 * Math.abs(s);
        } else if (this.activePose === 'Roll Over') {
            const s = Math.sin(t * 0.003);
            body.roll = 28 * s;
            joints.head_tilt = 12 * s;
            joints.tail_wag = 25 * s;
        } else if (this.activePose === 'Shake Off') {
            const s = Math.sin(t * 0.02);
            body.roll = 12 * s;
            joints.head_pan = 10 * s;
            joints.l_ear = -20 + 12 * s;
            joints.r_ear = 20 - 12 * s;
            joints.tail_wag = 20 * s;
        } else if (this.activePose === 'Curious Tilt') {
            joints.head_tilt += Math.sin(t * 0.002) * 5;
        } else if (this.activePose === 'Attack / Guard') {
            joints.head_tilt += Math.sin(t * 0.01) * 3;
            joints.tail_wag = Math.sin(t * 0.008) * 8;
        } else if (this.activePose === 'Stand') {
            const breathe = Math.sin(t * 0.0016);
            joints.tail_wag = breathe * 6;
            body.height += breathe * 2;
        }

        this.JOINTS.forEach(key => { joints[key] = this.clamp(key, joints[key]); });
        return { joints, body };
    }

    computeGait(dt, moveVector, gaitType, bodyOverride) {
        const { fwd = 0, strafe = 0, turn = 0 } = moveVector || {};
        const mode = gaitType || this.gaitType || 'trot';
        const magnitude = Math.min(1, Math.abs(fwd) + Math.abs(strafe) + Math.abs(turn));
        const speed = mode === 'run' ? 2.6 : (mode === 'trot' ? 1.9 : 1.4);
        this.walkPhase = (this.walkPhase + dt * speed * (0.4 + magnitude)) % 1;

        const joints = this.basePose();
        const body = Object.assign({ height: 0, roll: 0, pitch: 0, yaw: 0 }, bodyOverride || {});
        const phase = this.walkPhase;

        const offsets = {
            walk: { lf: 0.0, rr: 0.25, rf: 0.5, lr: 0.75 },
            trot: { lf: 0.0, rr: 0.0, rf: 0.5, lr: 0.5 },
            run: { lf: 0.0, rr: 0.08, rf: 0.5, lr: 0.58 },
            march: { lf: 0.0, rf: 0.5, lr: 0.5, rr: 0.0 }
        }[mode] || { lf: 0.0, rr: 0.0, rf: 0.5, lr: 0.5 };

        const stepAmp = (this.stepLength / 60) * 24 * (0.35 + 0.65 * magnitude);
        const liftAmp = (this.stepHeight / 35) * 32 * (0.35 + 0.65 * magnitude);

        this.LEGS.forEach(leg => {
            const p = (phase + offsets[leg]) % 1;
            const swing = Math.sin(2 * Math.PI * p);
            const lift = Math.max(0, Math.sin(2 * Math.PI * p));
            const sideSign = (leg === 'lf' || leg === 'lr') ? 1 : -1;

            joints[leg + '_hip_pitch'] = -stepAmp * swing * (fwd >= 0 ? 1 : -1);
            joints[leg + '_knee'] = 5 + liftAmp * lift;
            joints[leg + '_hip_yaw'] = strafe * 14 * sideSign;

            if (Math.abs(fwd) < 0.05 && Math.abs(strafe) < 0.05 && Math.abs(turn) > 0.05) {
                joints[leg + '_hip_pitch'] = stepAmp * 0.5 * swing * (leg === 'lf' || leg === 'lr' ? 1 : -1) * Math.sign(turn);
                joints[leg + '_knee'] = 5 + liftAmp * 0.5 * lift;
            }
        });

        joints.tail_wag = Math.sin(2 * Math.PI * phase * 2) * 28 * (0.4 + 0.6 * magnitude);
        joints.head_pan = turn * 15;
        joints.head_tilt = mode === 'run' ? -6 : 0;
        body.height = 8 * Math.sin(4 * Math.PI * phase) + (bodyOverride ? bodyOverride.height || 0 : 0);
        body.roll = strafe * 5;
        body.yaw = turn * 9;
        body.pitch = mode === 'run' ? 6 : 0;

        this.JOINTS.forEach(key => { joints[key] = this.clamp(key, joints[key]); });
        return { joints, body };
    }

    servoAngles(joints) {
        const result = {};
        this.JOINTS.forEach(name => {
            const logical = (joints[name] || 0) + (this.calibrationTrims[name] || 0);
            result[name] = Math.max(0, Math.min(180, 90 + logical));
        });
        return result;
    }
}

if (typeof window !== 'undefined') {
    window.DogKinematics = DogKinematics;
}

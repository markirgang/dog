/**
 * Dog Robot 3D Renderer (Three.js WebGL)
 * Procedural 17-DOF quadruped:
 *   head pan/tilt, tail wag, two ears and four 3-joint legs.
 * Includes eyes, L298 jaw and a live ground grid.
 */

class Dog3DRenderer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) throw new Error('Dog3D container not found: ' + containerId);

        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(
            42,
            Math.max(1, this.container.clientWidth) / Math.max(1, this.container.clientHeight),
            1,
            6000
        );
        this.camera.position.set(560, 420, 720);

        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this.renderer.domElement);

        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.06;
        this.controls.maxPolarAngle = Math.PI / 2 + 0.08;
        this.controls.target.set(0, 160, 0);

        this.mouthEnergized = false;
        this.neckPulsating = false;
        this.baseHeight = 230;
        this.currentJoints = null;
        this.currentBody = { height: 0, roll: 0, pitch: 0, yaw: 0 };
        this.mouthValue = 0;

        this.setupEnvironment();
        this.buildRobot();

        window.addEventListener('resize', () => this.onWindowResize());
        this.clock = new THREE.Clock();
        this.animate = this.animate.bind(this);
        requestAnimationFrame(this.animate);
    }

    setupEnvironment() {
        this.scene.background = new THREE.Color(0x0b0e14);
        this.scene.fog = new THREE.FogExp2(0x0b0e14, 0.00065);

        this.scene.add(new THREE.AmbientLight(0xffffff, 0.65));

        const dirLight = new THREE.DirectionalLight(0xffffff, 1.25);
        dirLight.position.set(360, 620, 320);
        dirLight.castShadow = true;
        dirLight.shadow.mapSize.width = 2048;
        dirLight.shadow.mapSize.height = 2048;
        dirLight.shadow.camera.near = 100;
        dirLight.shadow.camera.far = 2200;
        this.scene.add(dirLight);

        const rimLight = new THREE.PointLight(0x00f0ff, 1.6, 900);
        rimLight.position.set(-260, 320, -300);
        this.scene.add(rimLight);

        const warmLight = new THREE.PointLight(0xec4899, 1.1, 800);
        warmLight.position.set(300, 180, 260);
        this.scene.add(warmLight);

        const grid = new THREE.GridHelper(2200, 44, 0x00f0ff, 0x1e293b);
        grid.position.y = 0;
        this.scene.add(grid);

        const ground = new THREE.Mesh(
            new THREE.PlaneGeometry(2600, 2600),
            new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.85, metalness: 0.1 })
        );
        ground.rotation.x = -Math.PI / 2;
        ground.receiveShadow = true;
        this.scene.add(ground);
    }

    box(w, h, d, color, metalness, roughness) {
        const mesh = new THREE.Mesh(
            new THREE.BoxGeometry(w, h, d),
            new THREE.MeshStandardMaterial({
                color: color,
                metalness: metalness === undefined ? 0.65 : metalness,
                roughness: roughness === undefined ? 0.35 : roughness
            })
        );
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        return mesh;
    }

    cylinder(radiusTop, radiusBottom, height, color, segments) {
        const mesh = new THREE.Mesh(
            new THREE.CylinderGeometry(radiusTop, radiusBottom, height, segments || 16),
            new THREE.MeshStandardMaterial({ color: color, metalness: 0.7, roughness: 0.3 })
        );
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        return mesh;
    }

    buildRobot() {
        this.robotGroup = new THREE.Group();
        this.scene.add(this.robotGroup);

        this.bodyGroup = new THREE.Group();
        this.bodyGroup.position.y = this.baseHeight;
        this.robotGroup.add(this.bodyGroup);

        // Main body / chassis
        const body = this.box(155, 82, 310, 0x334155, 0.75, 0.3);
        body.position.y = 0;
        this.bodyGroup.add(body);

        const spine = this.box(130, 18, 330, 0x00f0ff, 0.5, 0.3);
        spine.position.y = 48;
        this.bodyGroup.add(spine);

        const belly = this.box(140, 16, 280, 0x0f172a, 0.9, 0.2);
        belly.position.y = -46;
        this.bodyGroup.add(belly);

        this.buildHead();
        this.buildTail();
        this.buildEars();
        this.buildLeg('lf');
        this.buildLeg('rf');
        this.buildLeg('lr');
        this.buildLeg('rr');
        this.buildSensors();
    }

    buildHead() {
        this.headPanGroup = new THREE.Group();
        this.headPanGroup.position.set(0, 38, 172);
        this.bodyGroup.add(this.headPanGroup);

        const neck = this.cylinder(22, 26, 36, 0x475569, 16);
        neck.position.set(0, 8, 8);
        neck.rotation.x = Math.PI / 2;
        this.headPanGroup.add(neck);

        this.headTiltGroup = new THREE.Group();
        this.headTiltGroup.position.set(0, 12, 30);
        this.headPanGroup.add(this.headTiltGroup);

        const head = this.box(112, 82, 96, 0x1e293b, 0.8, 0.25);
        head.position.set(0, 10, 18);
        this.headTiltGroup.add(head);

        const snout = this.box(72, 44, 62, 0x0284c7, 0.7, 0.3);
        snout.position.set(0, -2, 82);
        this.headTiltGroup.add(snout);

        const nose = this.box(28, 18, 14, 0x0f172a, 0.9, 0.15);
        nose.position.set(0, 6, 114);
        this.headTiltGroup.add(nose);

        const eyeMat = new THREE.MeshBasicMaterial({ color: 0x00f0ff });
        [-26, 26].forEach(x => {
            const eye = new THREE.Mesh(new THREE.SphereGeometry(8, 16, 16), eyeMat);
            eye.position.set(x, 24, 62);
            this.headTiltGroup.add(eye);
        });

        // L298 channel 1 driven jaw
        this.jawGroup = new THREE.Group();
        this.jawGroup.position.set(0, -16, 58);
        this.headTiltGroup.add(this.jawGroup);
        const jaw = this.box(58, 12, 58, 0x0f172a, 0.9, 0.2);
        jaw.position.set(0, -6, 24);
        this.jawGroup.add(jaw);

        const tongue = this.box(28, 6, 30, 0xec4899, 0.3, 0.5);
        tongue.position.set(0, 0, 30);
        this.jawGroup.add(tongue);
    }

    buildTail() {
        this.tailGroup = new THREE.Group();
        this.tailGroup.position.set(0, 36, -168);
        this.bodyGroup.add(this.tailGroup);

        const base = this.cylinder(13, 17, 28, 0x475569, 12);
        base.rotation.x = Math.PI / 2;
        base.position.z = -12;
        this.tailGroup.add(base);

        const seg1 = this.box(18, 18, 70, 0x0284c7, 0.7, 0.3);
        seg1.position.z = -55;
        this.tailGroup.add(seg1);

        const seg2 = this.box(14, 14, 60, 0x1e293b, 0.8, 0.25);
        seg2.position.z = -112;
        seg2.rotation.x = 0.22;
        this.tailGroup.add(seg2);

        const tip = this.box(18, 18, 24, 0x00f0ff, 0.5, 0.3);
        tip.position.set(0, 8, -142);
        this.tailGroup.add(tip);
    }

    buildEars() {
        this.lEarGroup = new THREE.Group();
        this.lEarGroup.position.set(-38, 48, 4);
        this.headTiltGroup.add(this.lEarGroup);
        const lEar = this.box(22, 52, 12, 0x0284c7, 0.7, 0.3);
        lEar.position.y = 24;
        lEar.rotation.z = 0.15;
        this.lEarGroup.add(lEar);

        this.rEarGroup = new THREE.Group();
        this.rEarGroup.position.set(38, 48, 4);
        this.headTiltGroup.add(this.rEarGroup);
        const rEar = this.box(22, 52, 12, 0x0284c7, 0.7, 0.3);
        rEar.position.y = 24;
        rEar.rotation.z = -0.15;
        this.rEarGroup.add(rEar);
    }

    buildLeg(leg) {
        const positions = {
            lf: [-70, 100], rf: [70, 100], lr: [-70, -100], rr: [70, -100]
        };
        const pos = positions[leg];
        const hipYaw = new THREE.Group();
        hipYaw.position.set(pos[0], -24, pos[1]);
        this.bodyGroup.add(hipYaw);

        const hipPitch = new THREE.Group();
        hipYaw.add(hipPitch);

        const hipBall = new THREE.Mesh(
            new THREE.SphereGeometry(22, 14, 14),
            new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8, roughness: 0.25 })
        );
        hipBall.castShadow = true;
        hipPitch.add(hipBall);

        const thigh = this.box(38, 104, 44, 0x0284c7, 0.7, 0.3);
        thigh.position.y = -58;
        hipPitch.add(thigh);

        const knee = new THREE.Group();
        knee.position.y = -112;
        hipPitch.add(knee);

        const kneeBall = new THREE.Mesh(
            new THREE.SphereGeometry(17, 14, 14),
            new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.8, roughness: 0.25 })
        );
        kneeBall.castShadow = true;
        knee.add(kneeBall);

        const shin = this.box(30, 104, 36, 0x1e293b, 0.8, 0.25);
        shin.position.y = -58;
        knee.add(shin);

        const foot = this.box(46, 22, 66, 0x00f0ff, 0.5, 0.3);
        foot.position.set(0, -116, 16);
        knee.add(foot);

        this[leg + 'HipYaw'] = hipYaw;
        this[leg + 'HipPitch'] = hipPitch;
        this[leg + 'Knee'] = knee;
    }

    buildSensors() {
        this.sensorPucks = {};
        const positions = {
            front: [0, 30, 156],
            rear: [0, 30, -156],
            left: [-78, 30, 0],
            right: [78, 30, 0]
        };
        Object.keys(positions).forEach(name => {
            const puck = new THREE.Mesh(
                new THREE.CylinderGeometry(9, 9, 8, 16),
                new THREE.MeshStandardMaterial({ color: 0x10b981, metalness: 0.5, roughness: 0.35 })
            );
            puck.position.set(positions[name][0], positions[name][1], positions[name][2]);
            if (name === 'left' || name === 'right') {
                puck.rotation.z = Math.PI / 2;
            } else {
                puck.rotation.x = Math.PI / 2;
            }
            this.bodyGroup.add(puck);
            this.sensorPucks[name] = puck;
        });
    }

    setPose(joints, body) {
        if (!joints) return;
        this.currentJoints = joints;
        this.currentBody = body || { height: 0, roll: 0, pitch: 0, yaw: 0 };
        const d = Math.PI / 180;

        this.bodyGroup.position.y = this.baseHeight + (this.currentBody.height || 0);
        this.bodyGroup.rotation.x = (this.currentBody.pitch || 0) * d;
        this.bodyGroup.rotation.z = (this.currentBody.roll || 0) * d;
        this.bodyGroup.rotation.y = (this.currentBody.yaw || 0) * d;

        this.headPanGroup.rotation.y = (joints.head_pan || 0) * d;
        this.headTiltGroup.rotation.x = (joints.head_tilt || 0) * d;
        this.tailGroup.rotation.y = (joints.tail_wag || 0) * d;
        this.lEarGroup.rotation.x = (joints.l_ear || 0) * d;
        this.rEarGroup.rotation.x = (joints.r_ear || 0) * d;

        ['lf', 'rf', 'lr', 'rr'].forEach(leg => {
            this[leg + 'HipYaw'].rotation.y = (joints[leg + '_hip_yaw'] || 0) * d;
            this[leg + 'HipPitch'].rotation.x = (joints[leg + '_hip_pitch'] || 0) * d;
            this[leg + 'Knee'].rotation.x = (joints[leg + '_knee'] || 0) * d;
        });
    }

    setHardwareState(mouthEnergized, neckPulsating) {
        this.mouthEnergized = !!mouthEnergized;
        this.neckPulsating = !!neckPulsating;
    }

    setSensorDistances(distances) {
        if (!distances) return;
        Object.keys(this.sensorPucks || {}).forEach(name => {
            const puck = this.sensorPucks[name];
            const distance = Number(distances[name] || 0);
            const color = distance < 35 ? 0xef4444 : (distance < 70 ? 0xf59e0b : 0x10b981);
            puck.material.color.setHex(color);
        });
    }

    onWindowResize() {
        if (!this.container) return;
        const w = Math.max(1, this.container.clientWidth);
        const h = Math.max(1, this.container.clientHeight);
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
    }

    resetCamera() {
        this.camera.position.set(560, 420, 720);
        this.controls.target.set(0, 160, 0);
        this.controls.update();
    }

    animate() {
        requestAnimationFrame(this.animate);
        const dt = Math.min(0.05, this.clock.getDelta());
        const t = performance.now();

        const targetMouth = this.mouthEnergized ? 0.55 : 0.0;
        this.mouthValue += (targetMouth - this.mouthValue) * Math.min(1, dt * 8);
        if (this.jawGroup) this.jawGroup.rotation.x = this.mouthValue;

        if (this.neckPulsating) {
            this.headPanGroup.position.y = 38 + Math.sin(t * 0.012) * 5;
        } else {
            this.headPanGroup.position.y += (38 - this.headPanGroup.position.y) * Math.min(1, dt * 6);
        }

        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}

if (typeof window !== 'undefined') {
    window.Dog3DRenderer = Dog3DRenderer;
}

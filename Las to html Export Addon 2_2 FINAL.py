bl_info = {
    "name": "LiDAR WebGL HTML Exporter",
    "author": "AI Assistant",
    "version": (2, 1, 0),
    "blender": (5, 2, 0),
    "location": "View3D > N-Panel > LiDAR",
    "description": "Export LiDAR point clouds with Advanced Navigation, Cameras, and High-Res Export",
    "category": "Import-Export",
}

import bpy
import numpy as np
import zlib
import base64
import os
import json
from bpy_extras.io_utils import ExportHelper

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{TITLE} - LiDAR Viewer</title>
    <style>
        body { margin: 0; overflow: hidden; background-color: #111; font-family: sans-serif; color: white; }
        
        /* INVISIBLE DRAG ZONES FOR PRECISE ROTATION */
        .drag-zone {
            position: absolute;
            z-index: 50;
        }
        #rollZone { 
            top: 0; left: 0; width: 7.5%; height: 100%; cursor: ns-resize; 
        }
        #pitchZone { 
            top: 0; right: 245px; width: 7.5%; height: 100%; cursor: ns-resize; 
        }
        #yawZone { 
            bottom: 54px; left: 7.5%; width: calc(100% - 245px - 7.5%); height: 7.5%; cursor: ew-resize; 
        }

        #loading {
            display: none; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
            color: #fff; font-size: 20px; background: rgba(0,0,0,0.9); padding: 20px 30px; 
            border-radius: 8px; border: 1px solid #555; z-index: 999; box-shadow: 0 0 20px rgba(0,0,0,0.5);
        }
        
        #footer {
            position: absolute; bottom: 0; width: 100%; background: rgba(0,0,0,0.85); 
            padding: 10px 0; display: flex; justify-content: center; align-items: center; gap: 15px; z-index: 100;
        }
        .btn-group {
            display: flex; gap: 2px; align-items: center; background: #222; padding: 4px; border-radius: 6px;
        }
        #footer button {
            cursor: pointer; padding: 8px 15px; font-weight: bold; font-size: 14px; 
            background: #444; color: white; border: none; border-radius: 4px; transition: background 0.2s;
        }
        #footer button:hover { background: #666; }
        .view-btn { padding: 8px 20px !important; }
        
        #camSelect {
            background: #222; color: white; border: none; padding: 8px 10px; 
            border-radius: 4px; font-size: 14px; font-weight: bold; cursor: pointer; outline: none;
        }
        
        #scaleBarContainer {
            display: none; position: absolute; bottom: 70px; left: 50%; transform: translateX(-50%);
            text-align: center; pointer-events: none; z-index: 90;
        }
        #scaleBarText { font-weight: bold; margin-bottom: 4px; font-size: 14px; }
        #scaleBarLine {
            border-bottom: 2px solid; border-left: 2px solid; border-right: 2px solid;
            height: 8px; width: 100%; box-sizing: border-box;
        }
    </style>
    <script type="importmap">
        {
            "imports": {
                "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
                "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/",
                "fflate": "https://unpkg.com/fflate@0.8.2/esm/browser.js"
            }
        }
    </script>
</head>
<body>
    <!-- DRAG ZONES -->
    <div id="rollZone" class="drag-zone" title="Leveling (Roll) - drag up/down"></div>
    <div id="pitchZone" class="drag-zone" title="Vertical Tilt (Pitch) - drag up/down"></div>
    <div id="yawZone" class="drag-zone" title="Horizontal Rotation (Yaw) - drag left/right"></div>

    <div id="loading">Decompressing Point Cloud... Please wait.</div>
    
    <div id="scaleBarContainer">
        <div id="scaleBarText">10 m</div>
        <div id="scaleBarLine"></div>
    </div>
    
    <div id="footer">
        <div class="btn-group">
            <button id="rotXPlus" title="Rotate +90° around local X axis">+90x</button>
            <button id="viewTop" class="view-btn">Top View</button>
            <button id="rotXMinus" title="Rotate -90° around local X axis">-90x</button>
        </div>
        
        <div class="btn-group">
            <button id="prevCam" title="Previous Camera (Left Arrow)">◀</button>
            <select id="camSelect"></select>
            <button id="nextCam" title="Next Camera (Right Arrow)">▶</button>
        </div>

        <div class="btn-group">
            <button id="rotZPlus" title="Rotate +90° around global Z axis">+90z</button>
            <button id="viewFront" class="view-btn">Front View</button>
            <button id="rotZMinus" title="Rotate -90° around global Z axis">-90z</button>
        </div>
    </div>
    
    <script type="module">
        import * as THREE from 'three';
        import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
        import { GUI } from 'three/addons/libs/lil-gui.module.min.js';
        import * as fflate from 'fflate';

        const FILE_NAME = "{TITLE}";
        let renderer, scene, cameraPersp, cameraOrtho, currentCamera, controls, material, pointsObj;
        let guiControllers = {};
        
        let cameraList =[];
        let currentCamIndex = 0;
        
        const measurements =[];
        let isMeasuring = false;
        let measureState = 0;
        let currentMeasure = null;
        const raycaster = new THREE.Raycaster();
        const mouse = new THREE.Vector2();

        const params = {
            pointSize: 0.1, opacity: 0.5, brightness: 0.8, aoStrength: 0.0,
            camera: 'Perspective',
            displayMode: 'RGB', // 'RGB', 'Intensity', 'Monochrome'
            clipGlobalMin: -1.0, clipGlobalMax: 3.0, cutGlobal: false, globalIntensity: 0.7,
            clipFront: 0.1, clipBack: 100.0, cutLocal: false, localIntensity: 0.0,
            measureMode: false, measureColor: '#ffcc00', clearMeasurements: () => clearMeasurements(),
            bgColor: '#111111', captureRes: 1, takeScreenshot: () => takeScreenshot()
        };

        const modeMap = { 'RGB': 0.0, 'Intensity': 1.0, 'Monochrome': 2.0 };

        async function decodeData(b64) {
            if (!b64) return null;
            return new Promise((resolve) => {
                const binaryString = atob(b64);
                const len = binaryString.length;
                const bytes = new Uint8Array(len);
                for (let i = 0; i < len; i++) {
                    bytes[i] = binaryString.charCodeAt(i);
                }
                resolve(fflate.unzlibSync(bytes));
            });
        }

        function initViewer() {
            THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

            renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
            renderer.setSize(window.innerWidth, window.innerHeight);
            renderer.setPixelRatio(window.devicePixelRatio);
            renderer.setClearColor(params.bgColor);
            document.body.appendChild(renderer.domElement);

            scene = new THREE.Scene();

            cameraPersp = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
            cameraOrtho = new THREE.OrthographicCamera(-10, 10, 10, -10, 0.1, 1000);
            currentCamera = cameraPersp;

            controls = new OrbitControls(currentCamera, renderer.domElement);

            if (!window.B64_COL && window.B64_INT) params.displayMode = 'Intensity';
            else if (!window.B64_COL && !window.B64_INT) params.displayMode = 'Monochrome';

            buildGUI();
            setupMeasurementEvents();
            setupDragZones();
            setupViewButtons();

            window.addEventListener('resize', onWindowResize);
            controls.addEventListener('change', updateScaleBar);
            
            animate();
            initData();
        }

        async function initData() {
            document.getElementById('loading').style.display = 'block';
            try {
                const posData = await decodeData(window.B64_POS);
                const colData = await decodeData(window.B64_COL);
                const intData = await decodeData(window.B64_INT);
                
                const positions = new Float32Array(posData.buffer, posData.byteOffset, posData.byteLength / 4);
                
                let colors;
                if (colData) {
                    colors = new Float32Array(colData.buffer, colData.byteOffset, colData.byteLength / 4);
                } else {
                    colors = new Float32Array((positions.length / 3) * 4).fill(1.0);
                }

                let intensities;
                if (intData) {
                    intensities = new Float32Array(intData.buffer, intData.byteOffset, intData.byteLength / 4);
                } else {
                    intensities = new Float32Array(positions.length / 3).fill(0.8);
                }

                document.getElementById('loading').style.display = 'none';
                loadPointCloudData(positions, colors, intensities);
            } catch (e) {
                document.getElementById('loading').innerText = "Error loading data!";
                console.error(e);
            }
        }

        function rotateCameraAxis(axis, angle) {
            const offset = new THREE.Vector3().subVectors(currentCamera.position, controls.target);
            offset.applyAxisAngle(axis, angle);
            currentCamera.position.copy(controls.target).add(offset);
            
            cameraPersp.up.applyAxisAngle(axis, angle);
            cameraOrtho.up.applyAxisAngle(axis, angle);
            
            controls.update();
            updateScaleBar();
        }

        function setupDragZones() {
            const createZone = (id, axisType, callback) => {
                const el = document.getElementById(id);
                let isDragging = false;
                let lastVal = 0;

                el.addEventListener('pointerdown', (e) => {
                    if (isMeasuring || e.button !== 0) return;
                    isDragging = true;
                    lastVal = axisType === 'x' ? e.clientX : e.clientY;
                    el.setPointerCapture(e.pointerId);
                    e.preventDefault(); 
                });

                el.addEventListener('pointermove', (e) => {
                    if (!isDragging) return;
                    const currentVal = axisType === 'x' ? e.clientX : e.clientY;
                    const delta = currentVal - lastVal;
                    lastVal = currentVal;
                    callback(delta);
                });

                el.addEventListener('pointerup', (e) => { isDragging = false; el.releasePointerCapture(e.pointerId); });
                el.addEventListener('pointercancel', (e) => { isDragging = false; el.releasePointerCapture(e.pointerId); });
            };

            createZone('rollZone', 'y', (delta) => {
                const angle = delta * 0.005;
                const viewDir = new THREE.Vector3().subVectors(controls.target, currentCamera.position).normalize();
                cameraPersp.up.applyAxisAngle(viewDir, angle);
                cameraOrtho.up.applyAxisAngle(viewDir, angle);
                controls.update();
                updateScaleBar();
            });

            createZone('pitchZone', 'y', (delta) => {
                const angle = delta * 0.005;
                const rightAxis = new THREE.Vector3().setFromMatrixColumn(currentCamera.matrixWorld, 0).normalize();
                rotateCameraAxis(rightAxis, -angle); // Reversed direction
            });

            createZone('yawZone', 'x', (delta) => {
                const angle = delta * 0.005;
                rotateCameraAxis(new THREE.Vector3(0, 0, 1), -angle); // Reversed direction
            });
        }

        function loadPointCloudData(positions, colors, intensities) {
            if (pointsObj) {
                scene.remove(pointsObj); pointsObj.geometry.dispose(); pointsObj.material.dispose();
            }
            clearMeasurements();

            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            geometry.setAttribute('customColor', new THREE.BufferAttribute(colors, 4));
            geometry.setAttribute('intensity', new THREE.BufferAttribute(intensities, 1));
            geometry.computeBoundingBox(); geometry.computeBoundingSphere();

            const center = geometry.boundingBox.getCenter(new THREE.Vector3());
            const radius = geometry.boundingSphere.radius;
            
            const maxZ = geometry.boundingBox.max.z + 0.1;
            const minZ = geometry.boundingBox.min.z - 0.1;

            material = new THREE.ShaderMaterial({
                uniforms: {
                    uPointSize: { value: params.pointSize }, 
                    uPixelRatio: { value: window.devicePixelRatio },
                    uOpacity: { value: params.opacity }, 
                    uBrightness: { value: params.brightness }, 
                    uAOStrength: { value: params.aoStrength },
                    uDisplayMode: { value: modeMap[params.displayMode] },
                    
                    uGlobalZMin: { value: params.clipGlobalMin }, 
                    uGlobalZMax: { value: params.clipGlobalMax },
                    uCutGlobal: { value: params.cutGlobal ? 1.0 : 0.0 }, 
                    uGlobalIntensity: { value: params.globalIntensity },
                    uClipStart: { value: params.clipFront }, 
                    uClipEnd: { value: params.clipBack },
                    uCutLocal: { value: params.cutLocal ? 1.0 : 0.0 }, 
                    uLocalIntensity: { value: params.localIntensity }
                },
                transparent: true, depthWrite: false,
                vertexShader: `
                    uniform float uPointSize; 
                    uniform float uPixelRatio; 
                    attribute vec4 customColor;
                    attribute float intensity;
                    
                    varying vec4 vColor; 
                    varying float vDepth; 
                    varying float vWorldZ;
                    varying float vIntensity;
                    
                    void main() {
                        vColor = customColor;
                        vIntensity = intensity;
                        
                        vec4 worldPosition = modelMatrix * vec4(position, 1.0); 
                        vWorldZ = worldPosition.z;
                        
                        vec4 mvPosition = viewMatrix * worldPosition; 
                        gl_Position = projectionMatrix * mvPosition;
                        
                        if (projectionMatrix[2][3] == -1.0) { 
                            gl_PointSize = uPointSize * uPixelRatio * (100.0 / -mvPosition.z); 
                        } else { 
                            gl_PointSize = uPointSize * uPixelRatio; 
                        }
                        vDepth = -mvPosition.z;
                    }
                `,
                fragmentShader: `
                    uniform float uOpacity; 
                    uniform float uBrightness; 
                    uniform float uAOStrength;
                    uniform float uDisplayMode;
                    
                    uniform float uGlobalZMin; 
                    uniform float uGlobalZMax; 
                    uniform float uCutGlobal; 
                    uniform float uGlobalIntensity;
                    
                    uniform float uClipStart; 
                    uniform float uClipEnd; 
                    uniform float uCutLocal; 
                    uniform float uLocalIntensity;
                    
                    varying vec4 vColor; 
                    varying float vDepth; 
                    varying float vWorldZ;
                    varying float vIntensity;
                    
                    void main() {
                        vec2 coord = gl_PointCoord - vec2(0.5); 
                        float dist = length(coord);
                        if (dist > 0.5) discard;
                        
                        if (uCutLocal > 0.5 && (vDepth < uClipStart || vDepth > uClipEnd)) discard;
                        if (uCutGlobal > 0.5 && (vWorldZ < uGlobalZMin || vWorldZ > uGlobalZMax)) discard;
                        
                        vec3 baseColor;
                        if (uDisplayMode < 0.5) {
                            baseColor = vColor.rgb;           // RGB
                        } else if (uDisplayMode < 1.5) {
                            baseColor = vec3(vIntensity);     // Intensity
                        } else {
                            baseColor = vec3(0.85);           // Monochrome
                        }
                        
                        vec3 finalColor = baseColor;
                        
                        // Z-Elevation Tint (Red to Cyan)
                        float normGlobal = clamp((vWorldZ - uGlobalZMin) / max(uGlobalZMax - uGlobalZMin, 0.001), 0.0, 1.0);
                        finalColor = mix(finalColor, mix(vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 1.0), normGlobal), pow(uGlobalIntensity, 0.4));
                        
                        // Depth Tint (Blue)
                        float normLocal = clamp((vDepth - uClipStart) / max(uClipEnd - uClipStart, 0.001), 0.0, 1.0);
                        finalColor = mix(finalColor, vec3(0.0, 0.5, 1.0), normLocal * pow(uLocalIntensity, 0.4));
                        
                        // Brightness Adjustment
                        if (uBrightness > 0.0) finalColor = mix(finalColor, vec3(1.0), uBrightness);
                        else if (uBrightness < 0.0) finalColor = mix(finalColor, vec3(0.0), -uBrightness);
                        
                        // Ambient Occlusion Simulation
                        if (uAOStrength > 0.0) finalColor *= mix(1.0, clamp(1.0 - (dist * 2.5), 0.0, 1.0), uAOStrength);
                        
                        gl_FragColor = vec4(finalColor, uOpacity);
                    }
                `
            });

            pointsObj = new THREE.Points(geometry, material);
            scene.add(pointsObj);

            const aspect = window.innerWidth / window.innerHeight;
            const maxRadius = Math.max(radius, 1.0);
            
            cameraPersp.near = maxRadius * 0.001; cameraPersp.far = maxRadius * 100; cameraPersp.updateProjectionMatrix();
            cameraOrtho.left = -radius * aspect; cameraOrtho.right = radius * aspect; cameraOrtho.top = radius; cameraOrtho.bottom = -radius;
            cameraOrtho.near = maxRadius * 0.001; cameraOrtho.far = maxRadius * 100; cameraOrtho.updateProjectionMatrix();

            const sliderMin = Math.min(minZ, -10);
            const sliderMax = Math.max(maxZ, 10);
            guiControllers.clipGlobalMin.min(sliderMin).max(sliderMax).setValue(minZ);
            guiControllers.clipGlobalMax.min(sliderMin).max(sliderMax).setValue(maxZ);
            
            guiControllers.clipFront.max(maxRadius * 2).setValue(0.1);
            guiControllers.clipBack.max(maxRadius * 4).setValue(maxRadius * 2);

            setupCameras(center, radius);
        }

        function setupCameras(center, radius) {
            cameraList =[];
            
            // 1. Isometric (Default)
            cameraList.push({
                name: "Isometric (Default)",
                isSpecial: true,
                setup: () => {
                    guiControllers.camera.setValue('Orthographic');
                    currentCamera.position.copy(center).add(new THREE.Vector3(0.7 * radius * 1.5, -0.7 * radius * 1.5, 0.7 * radius * 1.5));
                    currentCamera.up.set(0, 0, 1);
                    currentCamera.lookAt(center);
                    controls.target.copy(center);
                    
                    params.clipFront = 0.1;
                    params.clipBack = radius * 4;
                    updateCameraSettings();
                }
            });

            // 2. Blender Cameras
            if (window.CAMERAS_JSON) {
                window.CAMERAS_JSON.forEach(cam => {
                    cameraList.push({ name: cam.name, isSpecial: false, data: cam });
                });
            }

            const select = document.getElementById('camSelect');
            select.innerHTML = '';
            cameraList.forEach((c, i) => {
                const opt = document.createElement('option');
                opt.value = i;
                opt.innerText = c.name;
                select.appendChild(opt);
            });

            select.addEventListener('change', (e) => applyCamera(parseInt(e.target.value)));
            
            document.getElementById('prevCam').addEventListener('click', () => switchCamera(-1));
            document.getElementById('nextCam').addEventListener('click', () => switchCamera(1));

            window.addEventListener('keydown', (e) => {
                if (e.key === 'ArrowLeft') {
                    e.preventDefault();
                    switchCamera(-1);
                } else if (e.key === 'ArrowRight') {
                    e.preventDefault();
                    switchCamera(1);
                }
            });

            applyCamera(0);
        }

        function switchCamera(dir) {
            if (cameraList.length === 0) return;
            currentCamIndex = (currentCamIndex + dir + cameraList.length) % cameraList.length;
            document.getElementById('camSelect').value = currentCamIndex;
            applyCamera(currentCamIndex);
        }

        function applyCamera(index) {
            currentCamIndex = index;
            const cam = cameraList[index];
            
            if (cam.isSpecial) {
                cam.setup();
            } else {
                const data = cam.data;
                const mat = new THREE.Matrix4().fromArray(data.matrix);
                const pos = new THREE.Vector3();
                const quat = new THREE.Quaternion();
                const scale = new THREE.Vector3();
                mat.decompose(pos, quat, scale);

                const isOrtho = data.type === 'ORTHO';
                guiControllers.camera.setValue(isOrtho ? 'Orthographic' : 'Perspective');

                // Set position and rotation after GUI change (which copies old camera)
                currentCamera.position.copy(pos);
                currentCamera.quaternion.copy(quat);
                currentCamera.up.set(0, 0, 1);

                if (isOrtho) {
                    const aspect = window.innerWidth / window.innerHeight;
                    const s = data.ortho_scale / 2;
                    cameraOrtho.left = -s * aspect;
                    cameraOrtho.right = s * aspect;
                    cameraOrtho.top = s;
                    cameraOrtho.bottom = -s;
                    cameraOrtho.updateProjectionMatrix();
                } else {
                    cameraPersp.fov = THREE.MathUtils.radToDeg(data.fov);
                    cameraPersp.updateProjectionMatrix();
                }

                params.clipFront = data.clip_start;
                params.clipBack = data.clip_end;
                
                // Target halfway between clip planes
                const forward = new THREE.Vector3(0, 0, -1).applyQuaternion(quat);
                const dist = (data.clip_start + data.clip_end) / 2.0;
                controls.target.copy(pos).add(forward.multiplyScalar(dist));
                
                updateCameraSettings();
            }
            controls.update();
            updateScaleBar();
        }

        function updateCameraSettings() {
            if (guiControllers.clipFront) {
                if (params.clipFront > guiControllers.clipFront._max) guiControllers.clipFront.max(params.clipFront * 2);
                guiControllers.clipFront.setValue(params.clipFront);
            }
            if (guiControllers.clipBack) {
                if (params.clipBack > guiControllers.clipBack._max) guiControllers.clipBack.max(params.clipBack * 2);
                guiControllers.clipBack.setValue(params.clipBack);
            }
            if (material) {
                material.uniforms.uClipStart.value = params.clipFront;
                material.uniforms.uClipEnd.value = params.clipBack;
            }
        }

        function buildGUI() {
            const gui = new GUI({ title: 'LiDAR Options' });
            
            const ptFolder = gui.addFolder('Point Style');
            ptFolder.add(params, 'pointSize', 0.1, 10.0).name('Point Size').onChange(v => { if(material) material.uniforms.uPointSize.value = v; });
            ptFolder.add(params, 'opacity', 0.0, 1.0).name('Opacity').onChange(v => { if(material) material.uniforms.uOpacity.value = v; });
            ptFolder.add(params, 'brightness', -1.0, 1.0).name('Brightness').onChange(v => { if(material) material.uniforms.uBrightness.value = v; });
            ptFolder.add(params, 'aoStrength', 0.0, 1.0).name('AO Simulation').onChange(v => { if(material) material.uniforms.uAOStrength.value = v; });
            ptFolder.addColor(params, 'bgColor').name('Background Color').onChange(v => { renderer.setClearColor(v); updateScaleBar(); });
            
            guiControllers.camera = gui.add(params, 'camera', ['Perspective', 'Orthographic']).name('Camera').onChange(v => {
                const oldCam = currentCamera; currentCamera = v === 'Perspective' ? cameraPersp : cameraOrtho;
                currentCamera.position.copy(oldCam.position); 
                currentCamera.quaternion.copy(oldCam.quaternion);
                currentCamera.up.copy(oldCam.up); 
                controls.object = currentCamera; controls.update(); updateScaleBar();
            });

            guiControllers.displayMode = gui.add(params, 'displayMode',['RGB', 'Intensity', 'Monochrome']).name('Display Mode').onChange(v => {
                if(material) material.uniforms.uDisplayMode.value = modeMap[v];
            });

            const measFolder = gui.addFolder('📏 Measurement (Distance)');
            measFolder.add(params, 'measureMode').name('Enable Measuring').onChange(v => {
                isMeasuring = v; controls.enabled = !v;
                renderer.domElement.style.cursor = v ? 'crosshair' : 'default';
                if (!v && measureState === 1 && currentMeasure) {
                    scene.remove(currentMeasure.line); currentMeasure.div.remove();
                    measurements.pop(); measureState = 0; currentMeasure = null;
                }
            });
            measFolder.addColor(params, 'measureColor').name('Line Color').onChange(v => { measurements.forEach(m => { m.line.material.color.set(v); m.div.style.color = v; }); });
            measFolder.add(params, 'clearMeasurements').name('🗑️ Clear All');

            const glFolder = gui.addFolder('Global Clipping (Z Elevation)');
            guiControllers.clipGlobalMin = glFolder.add(params, 'clipGlobalMin', -10, 10).name('Z Bottom').onChange(v => { if(material) material.uniforms.uGlobalZMin.value = v; });
            guiControllers.clipGlobalMax = glFolder.add(params, 'clipGlobalMax', -10, 10).name('Z Top').onChange(v => { if(material) material.uniforms.uGlobalZMax.value = v; });
            glFolder.add(params, 'cutGlobal').name('Apply Global Clip').onChange(v => { if(material) material.uniforms.uCutGlobal.value = v ? 1.0 : 0.0; });
            glFolder.add(params, 'globalIntensity', 0.0, 1.0).name('Elevation Tint').onChange(v => { if(material) material.uniforms.uGlobalIntensity.value = v; });

            const locFolder = gui.addFolder('Local Clipping (Depth)');
            guiControllers.clipFront = locFolder.add(params, 'clipFront', 0.01, 100).name('Clip Front').onChange(v => { if(material) material.uniforms.uClipStart.value = v; });
            guiControllers.clipBack = locFolder.add(params, 'clipBack', 0.1, 100).name('Clip Back').onChange(v => { if(material) material.uniforms.uClipEnd.value = v; });
            locFolder.add(params, 'cutLocal').name('Apply Local Clip').onChange(v => { if(material) material.uniforms.uCutLocal.value = v ? 1.0 : 0.0; });
            locFolder.add(params, 'localIntensity', 0.0, 1.0).name('Depth Tint').onChange(v => { if(material) material.uniforms.uLocalIntensity.value = v; });

            const exportFolder = gui.addFolder('Image Export');
            exportFolder.add(params, 'captureRes', 1, 10, 1).name('Resolution Multiplier');
            exportFolder.add(params, 'takeScreenshot').name('📸 Save JPG Screenshot');
        }

        function clearMeasurements() {
            measurements.forEach(m => {
                scene.remove(m.line); if (m.div && m.div.parentNode) m.div.parentNode.removeChild(m.div);
            });
            measurements.length = 0; measureState = 0; currentMeasure = null;
        }

        function getFocalPlaneIntersection() {
            const planeNormal = new THREE.Vector3(0, 0, 1).applyQuaternion(currentCamera.quaternion);
            const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(planeNormal, controls.target);
            raycaster.setFromCamera(mouse, currentCamera);
            const target = new THREE.Vector3();
            if (raycaster.ray.intersectPlane(plane, target)) return target; return null;
        }

        function setupMeasurementEvents() {
            window.addEventListener('pointerdown', (e) => {
                if (!isMeasuring || e.button !== 0 || e.target.closest('#footer') || e.target.closest('.lil-gui') || e.target.classList.contains('drag-zone')) return;
                mouse.x = (e.clientX / window.innerWidth) * 2 - 1; mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
                const p = getFocalPlaneIntersection(); if (!p) return;

                if (measureState === 0) {
                    const geo = new THREE.BufferGeometry().setFromPoints([p, p.clone()]);
                    const mat = new THREE.LineBasicMaterial({ color: params.measureColor, depthTest: false, transparent: true });
                    const line = new THREE.Line(geo, mat); line.renderOrder = 999; scene.add(line);
                    const div = document.createElement('div');
                    Object.assign(div.style, {
                        position: 'absolute', color: params.measureColor, background: 'rgba(0,0,0,0.7)', padding: '2px 6px', borderRadius: '4px', fontFamily: 'sans-serif', fontSize: '14px',
                        fontWeight: 'bold', pointerEvents: 'none', transform: 'translate(-50%, -50%)', zIndex: '1000'
                    });
                    document.body.appendChild(div);
                    currentMeasure = { p1: p, p2: p.clone(), line, div }; measurements.push(currentMeasure); measureState = 1;
                } else if (measureState === 1) {
                    currentMeasure.p2.copy(p); updateMeasureGeometry(currentMeasure); measureState = 0; currentMeasure = null;
                }
            });
            window.addEventListener('pointermove', (e) => {
                if (!isMeasuring || measureState === 0) return;
                mouse.x = (e.clientX / window.innerWidth) * 2 - 1; mouse.y = -(e.clientY / window.innerHeight) * 2 + 1;
                const p = getFocalPlaneIntersection();
                if (p && currentMeasure) { currentMeasure.p2.copy(p); updateMeasureGeometry(currentMeasure); }
            });
        }

        function updateMeasureGeometry(m) {
            m.line.geometry.attributes.position.setXYZ(1, m.p2.x, m.p2.y, m.p2.z); m.line.geometry.attributes.position.needsUpdate = true;
            m.div.innerText = m.p1.distanceTo(m.p2).toFixed(3) + ' m';
        }

        function setupViewButtons() {
            const setView = (ox, oy, oz) => {
                if (!pointsObj) return;
                guiControllers.camera.setValue('Orthographic');
                const center = pointsObj.geometry.boundingBox.getCenter(new THREE.Vector3());
                const r = pointsObj.geometry.boundingSphere.radius;
                
                currentCamera.position.copy(center).add(new THREE.Vector3(ox * r * 1.5, oy * r * 1.5, oz * r * 1.5));
                
                if (ox === 0 && oy === 0 && oz === 1) currentCamera.up.set(0, 1, 0); 
                else currentCamera.up.set(0, 0, 1);
                
                currentCamera.lookAt(center); controls.target.copy(center); controls.update(); updateScaleBar();
            };
            
            document.getElementById('viewTop').onclick = () => setView(0, 0, 1);
            document.getElementById('viewFront').onclick = () => setView(0, -1, 0);

            document.getElementById('rotXPlus').onclick = () => {
                const rightAxis = new THREE.Vector3().setFromMatrixColumn(currentCamera.matrixWorld, 0).normalize();
                rotateCameraAxis(rightAxis, Math.PI / 2);
            };
            document.getElementById('rotXMinus').onclick = () => {
                const rightAxis = new THREE.Vector3().setFromMatrixColumn(currentCamera.matrixWorld, 0).normalize();
                rotateCameraAxis(rightAxis, -Math.PI / 2);
            };

            document.getElementById('rotZPlus').onclick = () => rotateCameraAxis(new THREE.Vector3(0, 0, 1), Math.PI / 2);
            document.getElementById('rotZMinus').onclick = () => rotateCameraAxis(new THREE.Vector3(0, 0, 1), -Math.PI / 2);
        }

        function getNegativeColor(hex) {
            if (hex.indexOf('#') === 0) hex = hex.slice(1);
            if (hex.length === 3) hex = hex[0]+hex[0]+hex[1]+hex[1]+hex[2]+hex[2];
            let r = (255 - parseInt(hex.slice(0, 2), 16)).toString(16).padStart(2, '0');
            let g = (255 - parseInt(hex.slice(2, 4), 16)).toString(16).padStart(2, '0');
            let b = (255 - parseInt(hex.slice(4, 6), 16)).toString(16).padStart(2, '0');
            return '#' + r + g + b;
        }

        function updateScaleBar() {
            const container = document.getElementById('scaleBarContainer');
            if (params.camera !== 'Orthographic' || !pointsObj) { container.style.display = 'none'; return; }
            container.style.display = 'block';

            const visibleWorldWidth = (cameraOrtho.right - cameraOrtho.left) / cameraOrtho.zoom;
            const targetWidth = visibleWorldWidth * 0.50;
            const magnitude = Math.pow(10, Math.floor(Math.log10(targetWidth)));
            let niceNumber = magnitude;
            if (targetWidth >= 5 * magnitude) niceNumber = 5 * magnitude;
            else if (targetWidth >= 2 * magnitude) niceNumber = 2 * magnitude;

            container.style.width = ((niceNumber / visibleWorldWidth) * 100) + '%';
            document.getElementById('scaleBarText').innerText = `${niceNumber} m`;
            
            const negColor = getNegativeColor(params.bgColor);
            document.getElementById('scaleBarText').style.color = negColor;
            document.getElementById('scaleBarLine').style.borderColor = negColor;
        }

        function takeScreenshot() {
            if(!material) return;
            const mult = parseInt(params.captureRes);
            const w = window.innerWidth, h = window.innerHeight;
            const gl = renderer.getContext();
            const maxRes = gl.getParameter(gl.MAX_RENDERBUFFER_SIZE);
            
            let pixelW = Math.floor(w * mult * window.devicePixelRatio); let pixelH = Math.floor(h * mult * window.devicePixelRatio);
            if (pixelW > maxRes || pixelH > maxRes) {
                const scaleDown = maxRes / Math.max(pixelW, pixelH); pixelW = Math.floor(pixelW * scaleDown); pixelH = Math.floor(pixelH * scaleDown);
            }

            const origPixelRatio = renderer.getPixelRatio(); const origAspect = currentCamera.aspect || (w / h);
            const [origLeft, origRight, origTop, origBottom] =[cameraOrtho.left, cameraOrtho.right, cameraOrtho.top, cameraOrtho.bottom];
            
            renderer.setPixelRatio(1.0); renderer.setSize(pixelW, pixelH, false); material.uniforms.uPixelRatio.value = window.devicePixelRatio; 
            
            const renderAspect = pixelW / pixelH;
            if (params.camera === 'Orthographic') {
                const orthoSize = (origTop - origBottom) / 2;
                cameraOrtho.left = -orthoSize * renderAspect; cameraOrtho.right = orthoSize * renderAspect; cameraOrtho.updateProjectionMatrix();
            } else { cameraPersp.aspect = renderAspect; cameraPersp.updateProjectionMatrix(); }

            renderer.render(scene, currentCamera);
            let finalName = FILE_NAME + (params.camera === 'Orthographic' ? '_Ortho' : '_Persp');
            const needsCanvas = (params.camera === 'Orthographic') || measurements.length > 0;

            if (needsCanvas) {
                const canvas2d = document.createElement('canvas'); canvas2d.width = pixelW; canvas2d.height = pixelH;
                const ctx2d = canvas2d.getContext('2d'); ctx2d.drawImage(renderer.domElement, 0, 0);

                measurements.forEach(m => {
                    const p1 = m.p1.clone().project(currentCamera); const p2 = m.p2.clone().project(currentCamera);
                    if (p1.z > 1.0 || p2.z > 1.0) return;
                    const[x1, y1] =[(p1.x * 0.5 + 0.5) * pixelW, (-p1.y * 0.5 + 0.5) * pixelH];
                    const[x2, y2] =[(p2.x * 0.5 + 0.5) * pixelW, (-p2.y * 0.5 + 0.5) * pixelH];

                    ctx2d.strokeStyle = params.measureColor; ctx2d.lineWidth = 2; ctx2d.beginPath(); ctx2d.moveTo(x1, y1); ctx2d.lineTo(x2, y2); ctx2d.stroke();
                    ctx2d.fillStyle = params.measureColor; ctx2d.beginPath(); ctx2d.arc(x1, y1, 4, 0, Math.PI*2); ctx2d.fill(); ctx2d.beginPath(); ctx2d.arc(x2, y2, 4, 0, Math.PI*2); ctx2d.fill();

                    const dist = m.p1.distanceTo(m.p2).toFixed(3) + ' m'; const[midX, midY] =[(x1 + x2) / 2, (y1 + y2) / 2];
                    ctx2d.font = 'bold 16px sans-serif'; ctx2d.textAlign = 'center'; ctx2d.textBaseline = 'middle';
                    const mText = ctx2d.measureText(dist); ctx2d.fillStyle = 'rgba(0,0,0,0.7)'; ctx2d.fillRect(midX - mText.width/2 - 6, midY - 14, mText.width + 12, 28);
                    ctx2d.fillStyle = params.measureColor; ctx2d.fillText(dist, midX, midY);
                });

                if (params.camera === 'Orthographic') {
                    const visibleWorldWidth = (cameraOrtho.right - cameraOrtho.left) / cameraOrtho.zoom;
                    const targetWidth = visibleWorldWidth * 0.50; const mag = Math.pow(10, Math.floor(Math.log10(targetWidth)));
                    const niceNumber = targetWidth >= 5 * mag ? 5 * mag : (targetWidth >= 2 * mag ? 2 * mag : mag);
                    const barWidthPixels = (niceNumber / visibleWorldWidth) * pixelW; const barX = pixelW / 2 - barWidthPixels / 2; const barY = pixelH - 10;
                    const negColor = getNegativeColor(params.bgColor);
                    
                    ctx2d.strokeStyle = negColor; ctx2d.lineWidth = 2; ctx2d.beginPath(); ctx2d.moveTo(barX, barY - 10); ctx2d.lineTo(barX, barY);
                    ctx2d.lineTo(barX + barWidthPixels, barY); ctx2d.lineTo(barX + barWidthPixels, barY - 10); ctx2d.stroke();
                    
                    ctx2d.fillStyle = negColor; ctx2d.font = 'bold 16px sans-serif'; ctx2d.textAlign = 'center'; ctx2d.fillText(`${niceNumber} m`, barX + barWidthPixels/2, barY - 15);
                    finalName += `_x_${visibleWorldWidth.toFixed(2).replace('.', ',')}m`;
                }

                const link = document.createElement('a'); link.download = `${finalName}.jpg`; link.href = canvas2d.toDataURL('image/jpeg', 0.95); link.click();
            } else {
                const link = document.createElement('a'); link.download = `${finalName}.jpg`; link.href = renderer.domElement.toDataURL('image/jpeg', 0.95); link.click();
            }
            
            renderer.setPixelRatio(origPixelRatio); renderer.setSize(w, h, false);
            if (params.camera === 'Orthographic') {
                cameraOrtho.left = origLeft; cameraOrtho.right = origRight; cameraOrtho.top = origTop; cameraOrtho.bottom = origBottom; cameraOrtho.updateProjectionMatrix();
            } else { cameraPersp.aspect = origAspect; cameraPersp.updateProjectionMatrix(); }
        }

        function onWindowResize() {
            const aspect = window.innerWidth / window.innerHeight;
            cameraPersp.aspect = aspect; cameraPersp.updateProjectionMatrix();
            const orthoSize = (cameraOrtho.top - cameraOrtho.bottom) / 2;
            cameraOrtho.left = -orthoSize * aspect; cameraOrtho.right = orthoSize * aspect; cameraOrtho.updateProjectionMatrix();
            renderer.setSize(window.innerWidth, window.innerHeight); updateScaleBar();
        }

        function animate() {
            requestAnimationFrame(animate); controls.update(); renderer.render(scene, currentCamera);
            measurements.forEach(m => {
                const mid = m.p1.clone().lerp(m.p2, 0.5); mid.project(currentCamera);
                if (mid.z > 1.0) { m.div.style.display = 'none'; } 
                else {
                    m.div.style.display = 'block';
                    m.div.style.left = ((mid.x * 0.5 + 0.5) * window.innerWidth) + 'px'; m.div.style.top = ((-mid.y * 0.5 + 0.5) * window.innerHeight) + 'px';
                }
            });
        }

        initViewer();
    </script>
    
    <script>
        window.B64_POS = "{B64_POS}";
        window.B64_COL = "{B64_COL}";
        window.B64_INT = "{B64_INT}";
        window.CAMERAS_JSON = {CAMERAS_JSON};
    </script>
</body>
</html>
"""

class EXPORT_OT_lidar_html(bpy.types.Operator, ExportHelper):
    """Export LiDAR with Measuring Tool, AO, and High-Res Export"""
    bl_idname = "export_scene.lidar_html"
    bl_label = "Export to HTML"
    bl_options = {'REGISTER', 'UNDO'}

    filename_ext = ".html"
    filter_glob: bpy.props.StringProperty(default="*.html", options={'HIDDEN'}, maxlen=255)

    # Priorytetowa lista nazw atrybutu koloru (Col - domyślna nazwa w importach LiDAR)
    COLOR_ATTR_NAMES = ["Col", "color", "Color", "col", "Cd", "rgb", "RGB"]

    def execute(self, context):
        obj = context.active_object

        # ── Obsługa MESH i POINTCLOUD ────────────────────────────────────────
        if not obj or obj.type not in {'MESH', 'POINTCLOUD'}:
            self.report({'ERROR'}, "Please select a Mesh or PointCloud object.")
            return {'CANCELLED'}

        self.report({'INFO'}, "Evaluating Geometry Nodes...")

        depsgraph = context.evaluated_depsgraph_get()
        eval_obj  = obj.evaluated_get(depsgraph)
        is_pc     = obj.type == 'POINTCLOUD'

        # ── 1. Ekstrakcja pozycji ────────────────────────────────────────────
        if is_pc:
            pc_data   = eval_obj.data
            num_verts = len(pc_data.points)

            if num_verts == 0:
                self.report({'ERROR'}, "No points visible to export!")
                return {'CANCELLED'}

            self.report({'INFO'}, f"Extracting {num_verts} points from PointCloud...")

            positions = np.empty(num_verts * 3, dtype=np.float32)
            # atrybut "position" to FLOAT_VECTOR — foreach_get używa klucza "vector"
            pc_data.attributes["position"].data.foreach_get("vector", positions)
            attrs = pc_data.attributes

        else:  # MESH
            mesh      = eval_obj.to_mesh()
            num_verts = len(mesh.vertices)

            if num_verts == 0:
                self.report({'ERROR'}, "No points visible to export!")
                eval_obj.to_mesh_clear()
                return {'CANCELLED'}

            self.report({'INFO'}, f"Extracting {num_verts} points from Mesh...")

            positions = np.empty(num_verts * 3, dtype=np.float32)
            mesh.vertices.foreach_get("co", positions)
            attrs = mesh.attributes

        pos_z   = zlib.compress(positions.tobytes(), level=9)
        b64_pos = base64.b64encode(pos_z).decode('ascii')

        # ── 2. Ekstrakcja koloru — obsługa "Col" i alternatywnych nazw ───────
        b64_col = ""
        if context.scene.lidar_export_colors:
            color_attr_name = next(
                (n for n in self.COLOR_ATTR_NAMES if n in attrs), None
            )
            if color_attr_name:
                self.report({'INFO'}, f"Reading colors from attribute: '{color_attr_name}'")
                colors = np.empty(num_verts * 4, dtype=np.float32)
                # FLOAT_COLOR / BYTE_COLOR — oba używają klucza "color"
                attrs[color_attr_name].data.foreach_get("color", colors)

                if np.max(colors) > 1.0:
                    colors /= 255.0

                col_z   = zlib.compress(colors.tobytes(), level=9)
                b64_col = base64.b64encode(col_z).decode('ascii')
            else:
                self.report({'WARNING'},
                    f"No color attribute found. Checked: {self.COLOR_ATTR_NAMES}")

        # ── 3. Ekstrakcja intensywności ──────────────────────────────────────
        b64_int = ""
        if context.scene.lidar_export_intensity and "intensity" in attrs:
            intensities = np.empty(num_verts, dtype=np.float32)
            attrs["intensity"].data.foreach_get("value", intensities)

            max_i = np.max(intensities)
            if max_i > 0.0:
                intensities = (intensities / max_i) ** 0.6

            int_z   = zlib.compress(intensities.tobytes(), level=9)
            b64_int = base64.b64encode(int_z).decode('ascii')

        # ── 4. Ekstrakcja kamer ──────────────────────────────────────────────
        cameras_data = []
        cams = [o for o in context.scene.objects if o.type == 'CAMERA']
        cams.sort(key=lambda x: x.name)

        for cam in cams:
            mat      = cam.matrix_world.transposed()
            mat_flat = [val for row in mat for val in row]
            cameras_data.append({
                "name":        cam.name,
                "type":        cam.data.type,
                "fov":         cam.data.angle,
                "ortho_scale": cam.data.ortho_scale,
                "clip_start":  cam.data.clip_start,
                "clip_end":    cam.data.clip_end,
                "matrix":      mat_flat
            })

        cameras_json = json.dumps(cameras_data)

        # Zwolnienie mesh tylko dla obiektów MESH
        if not is_pc:
            eval_obj.to_mesh_clear()

        self.report({'INFO'}, "Compressing data (this may take a moment)...")

        title        = os.path.splitext(os.path.basename(self.filepath))[0]
        html_content = HTML_TEMPLATE.replace("{TITLE}",        title)
        html_content = html_content.replace("{B64_POS}",       b64_pos)
        html_content = html_content.replace("{B64_COL}",       b64_col)
        html_content = html_content.replace("{B64_INT}",       b64_int)
        html_content = html_content.replace("{CAMERAS_JSON}",  cameras_json)

        with open(self.filepath, 'w', encoding='utf-8') as f:
            f.write(html_content)

        self.report({'INFO'}, f"Successfully exported {num_verts} points to HTML!")
        return {'FINISHED'}


class LIDAR_PT_ExportPanel(bpy.types.Panel):
    bl_label      = "LiDAR HTML WebGL"
    bl_idname     = "LIDAR_PT_ExportPanel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category   = "LiDAR"

    def draw(self, context):
        layout = self.layout
        obj    = context.active_object

        layout.label(text="Export Data Options:")
        layout.prop(context.scene, "lidar_export_colors")
        layout.prop(context.scene, "lidar_export_intensity")

        layout.separator()
        layout.label(text="Export to Standalone HTML:", icon='WORLD')

        row = layout.row()
        # Przycisk aktywny dla MESH i POINTCLOUD
        row.enabled = bool(obj and obj.type in {'MESH', 'POINTCLOUD'})
        row.operator("export_scene.lidar_html", text="Export HTML Viewer", icon='EXPORT')

classes = (EXPORT_OT_lidar_html, LIDAR_PT_ExportPanel)

def register():
    bpy.types.Scene.lidar_export_colors = bpy.props.BoolProperty(
        name="Export RGB Colors",
        description="Include vertex colors in the export (increases file size)",
        default=False
    )
    bpy.types.Scene.lidar_export_intensity = bpy.props.BoolProperty(
        name="Export Intensity",
        description="Include intensity values in the export (increases file size)",
        default=False
    )
    
    for cls in classes:
        bpy.utils.register_class(cls)

def unregister():
    del bpy.types.Scene.lidar_export_colors
    del bpy.types.Scene.lidar_export_intensity
    
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)

if __name__ == "__main__":
    register()
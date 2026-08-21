/* =========================================================
   FLIGHT TRAJECTORY SIMULATOR
   Application and Simulation Logic
   ========================================================= */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

const PROJECTILE_PRESETS = {
    'm982': {
        name: 'M982 Excalibur Shell',
        mass: 48,
        dragCoefficient: 0.3,
        referenceArea: 0.018,
    },
    'cannonball': {
        name: '18th Century Cannonball',
        mass: 5.4, // 12-pounder
        dragCoefficient: 0.47, // Sphere
        referenceArea: 0.0095, // 11cm diameter
    }
};

/**
 * Simulates the trajectory of a simple shell with constant mass.
 * @param {object} params - The parameters for the simulation.
 * @param {number} params.initialVelocity - The initial velocity of the shell (m/s).
 * @param {number} params.launchAngle - The launch angle in degrees.
 * @param {number} params.dragCoefficient - The drag coefficient.
 * @param {number} params.airDensity - The air density (kg/m^3).
 * @param {number} params.referenceArea - The reference area (m^2).
 * @param {number} params.gravity - Gravitational acceleration (m/s^2).
 * @param {number} params.mass - The mass of the shell (kg).
 * @param {number} params.timeStep - The time step for the simulation (s).
 * @returns {object} An object containing the full simulation data.
 */
function simulateProjectileTrajectory({ initialVelocity, launchAngle, dragCoefficient, airDensity, referenceArea, mass, gravity, timeStep }) {
    const g = gravity;

    // Convert launch angle to radians
    const launchAngleRad = (launchAngle * Math.PI) / 180;

    // Initial conditions
    let position = { x: 0.0, y: 0.0 };
    let velocity = {
        vx: initialVelocity * Math.cos(launchAngleRad),
        vy: initialVelocity * Math.sin(launchAngleRad)
    };

    const data = [{
        time: 0, ...position, ...velocity, speed: initialVelocity
    }];

    let time = 0;

    while (position.y >= 0) {
        const speed = Math.sqrt(velocity.vx ** 2 + velocity.vy ** 2);
        
        // Correctly calculate drag force based on total speed
        const dragForce = 0.5 * dragCoefficient * airDensity * referenceArea * speed ** 2;
        
        // Drag acceleration opposes the velocity vector
        const dragAccel = dragForce / mass;
        const ax = speed > 1e-6 ? -dragAccel * (velocity.vx / speed) : 0;
        const ay = speed > 1e-6 ? -g - (dragAccel * (velocity.vy / speed)) : -g;

        // Update velocities using Euler integration
        velocity.vx += ax * timeStep;
        velocity.vy += ay * timeStep;

        // Update positions
        position = {
            x: position.x + velocity.vx * timeStep,
            y: position.y + velocity.vy * timeStep
        };

        time += timeStep;

        data.push({
            time,
            ...position,
            ...velocity,
            speed: Math.sqrt(velocity.vx ** 2 + velocity.vy ** 2)
        });
    }

    return { data };
}


/**
 * V-2 ROCKET SIMULATION
 * This section contains all functions related to the V-2 rocket trajectory simulation.
 */

const V2_CONSTANTS = {
    g: 9.81, // m/s^2 at sea level
    earthRadius: 6371000, // meters
    R: 287.05, // Specific gas constant for dry air
    gamma: 1.4, // Heat capacity ratio for air
};

const V2_DRAG_COEFFICIENTS = {
    0.0: 0.15, 0.3: 0.16, 0.6: 0.18, 0.8: 0.24, 0.9: 0.33,
    1.0: 0.46, 1.1: 0.42, 1.2: 0.34, 1.5: 0.24, 2.0: 0.18,
    3.0: 0.15, 4.0: 0.14,
};
const machTable = Object.keys(V2_DRAG_COEFFICIENTS).map(Number);
const cdTable = Object.values(V2_DRAG_COEFFICIENTS);

/**
 * Linearly interpolates a value.
 * @param {number} x - The point to interpolate.
 * @param {number[]} xp - The x-coordinates of the data points.
 * @param {number[]} fp - The y-coordinates of the data points.
 * @returns {number} The interpolated value.
 */
function interp(x, xp, fp) {
    const i = xp.findIndex(val => val > x) - 1;
    if (i < 0) return fp[0];
    if (i >= xp.length - 1) return fp[fp.length - 1];
    return fp[i] + (fp[i + 1] - fp[i]) * ((x - xp[i]) / (xp[i + 1] - xp[i]));
}

/**
 * Calculates atmospheric density at a given altitude based on the International Standard Atmosphere model.
 * @param {number} h - Altitude in meters.
 * @returns {number} Air density in kg/m^3.
 */
function atmosphericDensity(h) {
    if (h < 11000) { // Troposphere
        const T = 288.15 - 0.0065 * h;
        const P = 101325 * (T / 288.15) ** 5.255876;
        return P / (V2_CONSTANTS.R * T);
    } else if (h < 20000) { // Lower Stratosphere
        const T = 216.65;
        const P = 22632 * Math.exp(-9.80665 * (h - 11000) / (V2_CONSTANTS.R * T));
        return P / (V2_CONSTANTS.R * T);
    } else { // Upper Stratosphere
        const rho20 = 0.08803;
        const H = 6500.0;
        return rho20 * Math.exp(-(h - 20000) / H);
    }
}

/**
 * Calculates the speed of sound at a given altitude.
 * @param {number} altitude - Altitude in meters.
 * @returns {number} Speed of sound in m/s.
 */
function calculateSpeedOfSound(altitude) {
    let T;
    if (altitude < 11000) {
        T = 288.15 - 0.0065 * altitude;
    } else { // altitude < 20000 or higher
        T = 216.65;
    }
    return Math.sqrt(V2_CONSTANTS.gamma * V2_CONSTANTS.R * T);
}

/**
 * Calculates the Mach number for a given velocity and altitude.
 * @param {number} velocity - The velocity of the object in m/s.
 * @param {number} altitude - The altitude in meters.
 * @returns {number} The Mach number.
 */
function calculateMachNumber(velocity, altitude) {
    const speedOfSound = calculateSpeedOfSound(altitude);
    return speedOfSound > 0 ? Math.abs(velocity) / speedOfSound : 0;
}

/**
 * Calculates the drag coefficient based on Mach number.
 * @param {number} velocity - The velocity of the object in m/s.
 * @param {number} altitude - The altitude in meters.
 * @returns {number} The drag coefficient.
 */
function calculateDragCoefficient(velocity, altitude) {
    let mach = calculateMachNumber(velocity, altitude);
    mach = Math.max(machTable[0], Math.min(mach, machTable[machTable.length - 1]));
    return interp(mach, machTable, cdTable);
}

/**
 * Calculates the total drag force on the rocket.
 * @param {number} velocity - The speed of the rocket in m/s.
 * @param {number} altitude - The altitude in meters.
 * @param {number} frontalArea - The frontal area of the rocket in m^2.
 * @returns {number} The drag force in Newtons.
 */
function calculateDragForce(velocity, altitude, frontalArea) {
    const density = atmosphericDensity(altitude);
    const cd = calculateDragCoefficient(velocity, altitude);
    return 0.5 * density * velocity ** 2 * cd * frontalArea;
}

/**
 * Calculates gravitational acceleration, adjusting for altitude.
 * @param {number} altitude - Altitude in meters.
 * @returns {number} Gravitational acceleration in m/s^2.
 */
function calculateGravity(altitude) {
    return V2_CONSTANTS.g * (V2_CONSTANTS.earthRadius / (V2_CONSTANTS.earthRadius + altitude)) ** 2;
}

/**
 * Determines the rocket's pitch angle (in degrees) based on time.
 * @param {number} time - Time since launch in seconds.
 * @returns {number} Pitch angle in degrees from the horizontal.
 */
function getPitchAngle(time, pitchStart, pitchEnd, startAngle, endAngle) {
    if (time < pitchStart) {
        return startAngle; // Initial pitch
    } else if (time < pitchEnd) {
        // Gravity turn program
        return interp(time, [pitchStart, pitchEnd], [startAngle, endAngle]);
    } else {
        return endAngle; // Final pitch
    }
}

/**
 * Simulates the trajectory of the V-2 rocket.
 * @param {object} params - The parameters for the simulation.
 * @param {number} params.startAngle - Initial pitch angle (degrees).
 * @param {number} params.endAngle - Final pitch angle (degrees).
 * @param {number} params.pitchStart - Time to start pitch program (s).
 * @param {number} params.pitchEnd - Time to end pitch program (s).
 * @param {number} params.timeStep - The time step for the simulation (s).
 * @param {number} params.maxTime - Maximum simulation time (s).
 * @param {number} params.initialMass - Total mass at launch (kg).
 * @param {number} params.endMass - Mass after fuel is spent (kg).
 * @param {number} params.burnTime - Duration of engine burn (s).
 * @param {number} params.frontalArea - Rocket's frontal area (m^2).
 * @param {number} params.thrust - Engine thrust in Newtons (N).
 * @param {number} [dt=0.1] - The time step for the simulation (s).
 * @param {function} [onProgress] - Optional callback for progress updates.
 * @returns {object} An object containing arrays for times, positions, velocities, and masses.
 */ 
function simulateV2Trajectory({ initialMass, endMass, burnTime, frontalArea, thrust, startAngle, endAngle, pitchStart, pitchEnd, timeStep, maxTime }) {
    const dt = timeStep;
    // Initial Conditions
    let position = { x: 0.0, y: 0.0 };
    let velocity = { vx: 0.0, vy: 0.0 };
    let mass = initialMass;
    const massFlowRate = (initialMass - endMass) / burnTime;
    let time = 0.0;

    // Data storage
    const data = [{
        time, ...position, ...velocity, speed: 0, mass, mach: 0, cd: V2_DRAG_COEFFICIENTS[0]
    }];

    let burnoutAltitude = -1;

    // Main simulation loop
    while (position.y >= 0) {
        // Determine current state
        const inBurnPhase = time < burnTime;
        const currentThrust = inBurnPhase ? thrust : 0.0;
        if (inBurnPhase) {
            mass = Math.max(endMass, initialMass - massFlowRate * time);
        }
        if (burnoutAltitude < 0 && time >= burnTime) {
            burnoutAltitude = position.y;
        }

        // --- Forces Calculation ---

        // Thrust Vector
        const pitchAngleRad = (getPitchAngle(time, pitchStart, pitchEnd, startAngle, endAngle) * Math.PI) / 180;
        const thrustVector = {
            x: currentThrust * Math.cos(pitchAngleRad),
            y: currentThrust * Math.sin(pitchAngleRad)
        };

        // Drag Vector
        const speed = Math.sqrt(velocity.vx ** 2 + velocity.vy ** 2);
        let dragVector = { x: 0, y: 0 };
        let dragForce = 0;
        if (speed > 1e-6) {
            dragForce = calculateDragForce(speed, position.y, frontalArea);
            dragVector = {
                x: -dragForce * (velocity.vx / speed),
                y: -dragForce * (velocity.vy / speed)
            };
        }

        // Gravity Vector
        const gravityVector = {
            x: 0.0,
            y: -mass * calculateGravity(position.y)
        };

        // Total Force and Acceleration
        const totalForce = {
            x: thrustVector.x + dragVector.x + gravityVector.x,
            y: thrustVector.y + dragVector.y + gravityVector.y
        };
        const acceleration = {
            x: totalForce.x / mass,
            y: totalForce.y / mass
        };

        // --- Euler Integration ---
        velocity.vx += acceleration.x * dt;
        velocity.vy += acceleration.y * dt;
        position.x += velocity.vx * dt;
        position.y += velocity.vy * dt;

        time += dt;

        // --- Data Storage & Progress ---
        const mach = calculateMachNumber(speed, position.y);
        const cd = calculateDragCoefficient(speed, position.y);
        data.push({
            time,
            ...position,
            ...velocity,
            speed,
            mass,
            mach,
            cd
        });
        
        // Safety break for very long or failed simulations
        if (time > maxTime) { 
            console.warn(`Simulation exceeded max time of ${maxTime}s, terminating.`);
            break;
        }
    }

    return { data, burnoutAltitude };
}


/* =========================================================
   UI AND APPLICATION LOGIC
   ========================================================= */

document.addEventListener('DOMContentLoaded', () => {
    // --- STATE ---
    let state = {
        simulationType: 'projectile', // 'projectile' or 'rocket'
        simulationData: null,
        target: null, // {x, y} for the game target
        animationFrameId: null,
        activeExplosions: [], // Holds our particle systems once spawned
        charts: {},
        isDarkMode: window.matchMedia('(prefers-color-scheme: dark)').matches
    };

    // --- DOM & 3D ELEMENTS ---
    let three = { scene: null, camera: null, renderer: null, controls: null, projectile: null, ground: null, line: null, scenery: null };

    const DOMElements = {
        themeToggle: document.getElementById('theme-toggle'),
        projectileTab: document.getElementById('projectile-tab'),
        rocketTab: document.getElementById('rocket-tab'),
        projectilePanel: document.getElementById('projectile-panel'),
        rocketPanel: document.getElementById('rocket-panel'),
        projectileType: document.getElementById('projectile-type'),
        view3DToggle: null, // Will be created dynamically
        projectileForm: document.getElementById('projectile-form'),
        rocketForm: document.getElementById('rocket-form'),
        runButton: document.getElementById('run-simulation'),
        resetButton: document.getElementById('reset-simulation'),
        exportButton: document.getElementById('export-csv'),
        formError: document.getElementById('form-error'),
        statusBadge: document.getElementById('simulation-status'),
        rocketOnlyElements: document.querySelectorAll('.rocket-only'),
        summary: {
            maxAltitude: document.getElementById('maximum-altitude'),
            // Rocket live readouts
            rocketFlightPhase: document.getElementById('rocket-flight-phase'),
            rocketLiveReadout: document.getElementById('rocket-live-readout'),
            rocketPowerOutput: document.getElementById('rocket-power-output'),
            rocketPowerSlider: document.getElementById('rocket-power'),
            // Summary cards
            range: document.getElementById('horizontal-range'),
            flightTime: document.getElementById('flight-time'),
            maxSpeed: document.getElementById('maximum-speed'),
            maxMach: document.getElementById('maximum-mach'),
            burnoutAltitude: document.getElementById('burnout-altitude'),
        },
        table: {
            head: document.getElementById('simulation-table-head'),
            body: document.getElementById('simulation-table-body'),
            limit: document.getElementById('table-row-limit'),
        },
        resetGraphButton: document.getElementById('reset-graph-button'),
        chartResetButtons: document.querySelectorAll('.chart-reset-button'),
        profileButtons: document.querySelectorAll('.profile-button'),
        animationCanvas: document.getElementById('animation-canvas'),
        get animationCtx() {
            return this.animationCanvas.getContext('2d');
        },
        threeCanvas: document.getElementById('three-canvas'),
        trajectoryChartCanvas: document.getElementById('trajectory-chart'),
    };

    // --- CHART CONFIGURATION ---
    const chartConfigs = {
        trajectory: {
            type: 'scatter',
            options: {
                scales: {
                    x: {
                        title: { display: true, text: 'Range (m)' },
                        min: 0,
                        max: 20000 // Set a larger default max
                    },
                    y: {
                        title: { display: true, text: 'Altitude (m)' },
                        min: 0,
                        max: 20000 // Set a larger default max
                    }
                }
            }
        },
        velocity: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Velocity (m/s)' } } } }
        },
        altitude: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Altitude (m)' } } } }
        },
        mass: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Mass (kg)' } } } }
        },
        mach: {
            type: 'line',
            options: { scales: { x: { title: { display: true, text: 'Time (s)' } }, y: { title: { display: true, text: 'Value' } } } }
        }
    };

    const chartColors = {
        blue: 'rgba(33, 85, 205, 0.8)',
        sky: 'rgba(100, 150, 255, 0.8)',
        green: 'rgba(22, 121, 79, 0.8)',
        orange: 'rgba(255, 159, 64, 0.8)',
        red: 'rgba(255, 99, 132, 0.8)',
    };

    // --- INITIALIZATION ---
    function init() {
        setupEventListeners();
        applyTheme(state.isDarkMode);
        populateProjectilePresets();
        create3DViewToggle();
        resetUI();
        updateCharts([]); // Create empty charts on load so target can be set
    }

    function create3DViewToggle() {
        const container = document.createElement('div');
        container.className = 'input-group full-width';
        container.style.flexDirection = 'row';
        container.style.alignItems = 'center';
        container.style.gap = '10px';
        container.style.marginTop = '0.5rem';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = 'view-3d-toggle';
        DOMElements.view3DToggle = checkbox;

        const label = document.createElement('label');
        label.htmlFor = 'view-3d-toggle';
        label.textContent = 'Enable 3D View';
        label.style.fontWeight = 'bold';

        container.append(checkbox, label);
        DOMElements.projectileForm.querySelector('.input-grid').prepend(container);
    }
    // --- EVENT LISTENERS ---
    function setupEventListeners() {
        DOMElements.trajectoryChartCanvas.addEventListener('click', onChartClick);
        DOMElements.themeToggle.addEventListener('click', toggleTheme);
        DOMElements.projectileTab.addEventListener('click', () => switchSimulationType('projectile'));
        DOMElements.rocketTab.addEventListener('click', () => switchSimulationType('rocket'));
        DOMElements.runButton.addEventListener('click', runSimulation);
        DOMElements.resetButton.addEventListener('click', resetUI);
        DOMElements.exportButton.addEventListener('click', exportToCSV);
        DOMElements.table.limit.addEventListener('change', () => updateTable(state.simulationData));
        DOMElements.resetGraphButton.addEventListener('click', resetGraph);
        DOMElements.projectileType.addEventListener('change', onProjectileTypeChange);
        DOMElements.chartResetButtons.forEach(button => {
            button.addEventListener('click', () => {
                const chartId = button.dataset.chart;
                if (state.charts[chartId]) {
                    state.charts[chartId].resetZoom();
                }
            });
        });
        DOMElements.profileButtons.forEach(button => {
            button.addEventListener('click', onProfileButtonClick);
        });
        DOMElements.summary.rocketPowerSlider.addEventListener('input', onPowerSliderChange);
        document.addEventListener('keydown', onKeyDown);
    }

    function onChartClick(event) {
        const chart = state.charts.trajectory;
        if (!chart || state.animationFrameId) return; // Don't set target while running
    
        const rect = chart.canvas.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;
    
        // Convert pixel coordinates to data coordinates
        const dataX = chart.scales.x.getValueForPixel(x);
        const dataY = chart.scales.y.getValueForPixel(y);
    
        // Ensure target is within the valid gameplay area
        if (dataX < 0 || dataY < 0) return;
    
        state.target = { x: dataX, y: dataY };
        drawTarget(); // Draw the new target immediately
    }

    function onProfileButtonClick(event) {
        const button = event.currentTarget;
        const profile = button.dataset.profile;

        DOMElements.profileButtons.forEach(btn => btn.classList.remove('active'));
        button.classList.add('active');

        document.getElementById('rocket-profile').value = profile;
        // You might want to update some rocket parameters here based on the profile
    }

    function onPowerSliderChange(event) {
        DOMElements.summary.rocketPowerOutput.textContent = `${event.currentTarget.value}%`;
    }

    function onKeyDown(event) {
        // OrbitControls handles camera movement, so this is no longer needed.
    }

    function resetGraph() {
        state.target = null;
        state.simulationData = null;
        resetUI(false); // Soft reset
        updateCharts([]); // Redraw empty charts
    }

    function populateProjectilePresets() {
        const select = DOMElements.projectileType;
        select.innerHTML = '';
        for (const key in PROJECTILE_PRESETS) {
            const option = document.createElement('option');
            option.value = key;
            option.textContent = PROJECTILE_PRESETS[key].name;
            select.appendChild(option);
        }
        onProjectileTypeChange(); // Set initial values
    }

    function onProjectileTypeChange() {
        updateProjectileInputs();
    }

    // --- THEME ---
    function toggleTheme() {
        state.isDarkMode = !state.isDarkMode;
        applyTheme(state.isDarkMode);
    }

    function applyTheme(isDark) {
        document.body.classList.toggle('dark-theme', isDark);
        DOMElements.themeToggle.setAttribute('aria-pressed', isDark);
        DOMElements.themeToggle.textContent = isDark ? 'Light mode' : 'Dark mode';
        // Re-render charts with correct colors if they exist
        Object.values(state.charts).forEach(chart => chart.destroy());
        state.charts = {};
        if (state.simulationData) {
            updateCharts(state.simulationData, true);
        }
    }

    // --- SIMULATION TYPE ---
    function switchSimulationType(type) {
        state.simulationType = type;
        const isProjectile = type === 'projectile';

        DOMElements.projectileTab.classList.toggle('active', isProjectile);
        DOMElements.projectileTab.setAttribute('aria-selected', isProjectile);
        DOMElements.rocketTab.classList.toggle('active', !isProjectile);
        DOMElements.rocketTab.setAttribute('aria-selected', !isProjectile);

        DOMElements.projectilePanel.hidden = !isProjectile;
        DOMElements.rocketPanel.hidden = isProjectile;

        DOMElements.rocketOnlyElements.forEach(el => el.hidden = isProjectile);

        resetUI();
    }

    // --- SIMULATION EXECUTION ---
    function runSimulation() {
        if (state.animationFrameId) { // If animation is running, stop it
            cancelAnimationFrame(state.animationFrameId);
            state.animationFrameId = null;
            setUIState('ready');
            return;
        }

        const form = state.simulationType === 'projectile' ? DOMElements.projectileForm : DOMElements.rocketForm;
        const params = getAndValidateFormParams(form);

        if (!params) return;

        setUIState('running');
        resetUI(false); // Soft reset to clear previous run visuals

        // Initialize charts before starting the animation
        updateCharts([]);

        // Use setTimeout to allow the UI to update before the heavy computation
        setTimeout(() => {
            try {
                const is3D = state.simulationType === 'projectile' && DOMElements.view3DToggle.checked;
                if (is3D) {
                    DOMElements.trajectoryChartCanvas.style.display = 'none';
                    DOMElements.threeCanvas.style.display = 'block';
                    init3DScene();
                } else {
                    DOMElements.trajectoryChartCanvas.style.display = 'block';
                    DOMElements.threeCanvas.style.display = 'none';
                }

                // Run the appropriate simulation to get the static data array first.
                const simulationResult = state.simulationType === 'projectile'
                    ? simulateProjectileTrajectory(params)
                    : simulateV2Trajectory(params);

                // Start the frame-by-frame playback animation.
                playbackAnimation(simulationResult);

            } catch (error) {
                console.error("Simulation failed:", error);
                setUIState('error', 'Simulation failed. Check console for details.');
            }
        }, 50);
    }

    /**
     * Animates a pre-calculated trajectory frame by frame.
     * @param {object} simulation - The result object from a simulation function.
     * @param {Array<object>} simulation.data - The array of trajectory data points.
     */
    function playbackAnimation(simulation) {
        const trajectoryData = simulation.data;
        if (!trajectoryData || trajectoryData.length === 0) {
            setUIState('error', 'Simulation produced no data.');
            return;
        }

        const is3D = state.simulationType === 'projectile' && DOMElements.view3DToggle.checked;
        const isProjectile = state.simulationType === 'projectile';
        state.simulationData = trajectoryData; // Store full data for other UI components

        // --- Playback state tracking ---
        let currentFrameIndex = 0;

        const trajectoryPoints = [];

        // --- Chart references ---
        const trajectoryChart = state.charts.trajectory;
        const velocityChart = state.charts.velocity;
        const altitudeChart = state.charts.altitude;
        const massChart = isProjectile ? null : state.charts.mass;
        const machChart = isProjectile ? null : state.charts.mach;

        // --- Live UI update helper ---
        function updateLiveUI(dataPoint, maxValues) {
            updateLiveSummary(dataPoint, maxValues.maxAltitude, maxValues.maxSpeed, maxValues.maxMach);
            if (!isProjectile) {
                // Use a reasonable default for burnTime if not in simulation data
                const burnTime = simulation.data[simulation.data.length - 1]?.burnTime || 65;
                const phase = dataPoint.time < burnTime ? "Boost Phase" : "Coast Phase";
                const readout = `Alt: ${dataPoint.y.toFixed(0)}m, Vel: ${dataPoint.speed.toFixed(1)}m/s`;
                DOMElements.summary.rocketFlightPhase.textContent = phase;
                DOMElements.summary.rocketLiveReadout.textContent = readout;
            }
        }
        
        // Pre-calculate max values for smoother live updates
        const maxValues = {
            maxAltitude: Math.max(...trajectoryData.map(p => p.y)),
            maxSpeed: Math.max(...trajectoryData.map(p => p.speed)),
            maxMach: isProjectile ? 0 : Math.max(...trajectoryData.map(p => p.mach || 0)),
        };

        function animationLoop() {
            state.animationFrameId = requestAnimationFrame(animationLoop);

            // Always update the 3D scene if it's active
            if (is3D) three.renderer.render(three.scene, three.camera);

            // SCENARIO A: Projectile is flying
            if (currentFrameIndex < trajectoryData.length) {
                const frameData = trajectoryData[currentFrameIndex];

                // Update UI and visuals
                updateLiveUI(frameData, maxValues);
                if (is3D) {
                    trajectoryPoints.push(new THREE.Vector3(frameData.x, frameData.y, 0));
                    update3DScene(frameData, trajectoryPoints);
                } else {
                    drawAnimatedProjectile(frameData, trajectoryChart);
                }

                currentFrameIndex++;

                // IMPACT CHECK: Trigger on the exact frame the loop hits the final index
                if (currentFrameIndex === trajectoryData.length) {
                    setUIState('complete');
                    updateSummary(simulation);
                    updateTable(trajectoryData);

                    if (is3D) {
                        // Hide the shell mesh
                        three.projectile.visible = false;
                        // Spawn a final blast at the impact point
                        const finalPos = trajectoryData[trajectoryData.length - 1];
                        const terrainHeight = getTerrainHeight(finalPos.x, 0);
                        state.activeExplosions.push(createBlast(finalPos.x, terrainHeight, 0));
                    }
                }
            }

            // SCENARIO B: Projectile has hit; animate the active particle system
            // Animate all active explosions and clean up faded ones
            state.activeExplosions = state.activeExplosions.filter(explosion => {
                if (explosion.material.opacity <= 0) {
                    // Cleanup memory
                    three.scene.remove(explosion.mesh);
                    explosion.geometry.dispose();
                    explosion.material.dispose();
                    return false; // Remove from array
                }

                const positionsAttr = explosion.geometry.attributes.position.array;

                for (let i = 0; i < explosion.count; i++) {
                    // Expand individual particle vectors outward
                    positionsAttr[i * 3] += explosion.velocities[i].x * 0.1;
                    positionsAttr[i * 3 + 1] += explosion.velocities[i].y * 0.1;
                    positionsAttr[i * 3 + 2] += explosion.velocities[i].z * 0.1;
                    // Pull particles downward with simulated gravity
                    explosion.velocities[i].y -= 2.5;
                }
                explosion.geometry.attributes.position.needsUpdate = true;

                // Smoothly fade out opacity
                explosion.material.opacity -= 0.015;
                return true; // Keep in array
            });

            // If playback is finished and there's no explosion, stop the loop.
            // For 3D, we keep the loop running for camera controls.
            if (currentFrameIndex >= trajectoryData.length && state.activeExplosions.length === 0 && !is3D) {
                cancelAnimationFrame(state.animationFrameId);
                state.animationFrameId = null;
                
                // Final UI updates for 2D mode
                setUIState('complete');
                updateSummary(simulation);
                updateTable(trajectoryData);
            } else if (is3D) {
                three.controls.update(); // Keep controls interactive
            }
        }

        // Populate charts with full data at the start for a static line
        updateCharts(trajectoryData, true);
        if (!is3D) {
            // For 2D, clear the trajectory line so we can animate it point by point
            state.charts.trajectory.data.datasets[0].data = [];
            state.charts.trajectory.update('none');
        }

        animationLoop();
    }

    /**
     * Calculates the terrain height at a given world position.
     * This function must match the terrain generation logic in init3DScene.
     * @param {number} x - The world x-coordinate.
     * @param {number} z - The world z-coordinate.
     * @returns {number} The terrain height (y-coordinate) at that point.
     */
    function getTerrainHeight(x, z) {
        const height = 1500 * (Math.sin(x / 5000) * Math.cos(z / 5000));
        return height - 500; // Matches the ground's y-position offset
    }

    function getAndValidateFormParams(form) {
        const formData = new FormData(form);
        const params = {};
        let isValid = true;
        let errorMessage = '';

        // For projectile, add preset values to params
        if (state.simulationType === 'projectile') {
            const presetKey = DOMElements.projectileType.value;
            if (PROJECTILE_PRESETS[presetKey]) {
                const preset = PROJECTILE_PRESETS[presetKey];
                // These are now read from the advanced inputs
            }
        }

        for (const [name, value] of formData.entries()) {
            const input = form.elements[name];

            // Skip numeric validation for non-numeric fields
            if (['projectileType', 'rocketPreset', 'rocketProfile'].includes(name)) {
                continue;
            }

            const numValue = parseFloat(value);

            if (isNaN(numValue) || value.trim() === '') {
                errorMessage = `Invalid value for ${input.labels[0].textContent}. Must be a number.`;
                isValid = false;
                break;
            }

            const min = parseFloat(input.min);
            const max = parseFloat(input.max);

            if (!isNaN(min) && numValue < min) {
                errorMessage = `${input.labels[0].textContent} must be at least ${min}.`;
                isValid = false;
                break;
            }
            if (!isNaN(max) && numValue > max) {
                errorMessage = `${input.labels[0].textContent} must not exceed ${max}.`;
                isValid = false;
                break;
            }

            params[name] = numValue;
        }

        // Specific cross-field validation
        if (params.endMass && params.initialMass && params.endMass >= params.initialMass) {
            errorMessage = 'Final mass must be less than initial mass.';
            isValid = false;
        }
        if (params.pitchStart && params.pitchEnd && params.pitchStart >= params.pitchEnd) {
            errorMessage = 'Pitch program start time must be before end time.';
            isValid = false;
        }

        DOMElements.formError.textContent = errorMessage;
        DOMElements.formError.hidden = isValid;

        return isValid ? params : null;
    }

    // --- UI STATE & RESET ---
    function setUIState(status, message = '') {
        const isSimulating = status === 'running';
        DOMElements.runButton.disabled = false; // Always enable run/stop
        DOMElements.resetButton.disabled = isSimulating;
        DOMElements.exportButton.disabled = isSimulating || !state.simulationData;

        // Disable forms during simulation
        DOMElements.projectileForm.disabled = isSimulating;
        DOMElements.rocketForm.disabled = isSimulating;

        DOMElements.statusBadge.classList.remove('success', 'error', 'running');
        switch (status) {
            case 'running':
                DOMElements.statusBadge.textContent = 'Running...';
                DOMElements.statusBadge.classList.add('running');
                DOMElements.runButton.textContent = 'Stop Simulation';
                DOMElements.runButton.classList.add('stop-button');
                break;
            case 'complete':
                DOMElements.statusBadge.textContent = 'Complete';
                DOMElements.statusBadge.classList.add('success');
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                break;
            case 'hit':
                DOMElements.statusBadge.textContent = 'Target Hit!';
                DOMElements.statusBadge.classList.add('success');
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                break;
            case 'error':
                if (state.animationFrameId) {
                    cancelAnimationFrame(state.animationFrameId);
                    state.animationFrameId = null;
                }
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                DOMElements.statusBadge.textContent = 'Error';
                DOMElements.statusBadge.classList.add('error');
                DOMElements.formError.textContent = message;
                DOMElements.formError.hidden = false;
                break;
            default: // 'ready'
                DOMElements.statusBadge.textContent = 'Ready';
                DOMElements.runButton.textContent = 'Run Simulation';
                DOMElements.runButton.classList.remove('stop-button');
                break;
        }
    }

    function resetUI(hardReset = true) {
        if (state.animationFrameId) {
            cancelAnimationFrame(state.animationFrameId);
            state.animationFrameId = null;
        }
        state.activeExplosions = [];
        state.simulationData = null;
        if (hardReset) state.target = null;

        // Reset canvases
        DOMElements.trajectoryChartCanvas.style.display = 'block';
        DOMElements.threeCanvas.style.display = 'none';
        if (three.renderer) {
            three.renderer.dispose();
            three.renderer = null;
            if (three.controls) three.controls.dispose();
            DOMElements.threeCanvas.innerHTML = '';
        }

        DOMElements.animationCtx.clearRect(0, 0, DOMElements.animationCanvas.width, DOMElements.animationCanvas.height);
        
        // Reset rocket live readouts
        if (state.simulationType === 'rocket') {
            DOMElements.summary.rocketFlightPhase.textContent = 'Ready for launch';
            DOMElements.summary.rocketLiveReadout.textContent = 'Select a profile and run the simulation.';
        }

        // Reset summary cards
        Object.values(DOMElements.summary).forEach(el => el.textContent = '—');

        // Clear charts
        Object.values(state.charts).forEach(chart => chart.destroy());
        state.charts = {};

        // Clear table
        DOMElements.table.body.innerHTML = `<tr><td colspan="8" class="empty-table-message">Run a simulation to view calculated data.</td></tr>`;
        updateTableHeaders();

        if (hardReset) {
            // DOMElements.projectileForm.reset(); // Can be annoying
            // DOMElements.rocketForm.reset();
            if (state.target) drawTarget(); // Redraw target if it exists
            updateProjectileInputs(); // Restore defaults
        }

        DOMElements.formError.hidden = true;
        setUIState('ready');
    }
    
    function redrawOverlay() {
        const ctx = DOMElements.animationCtx;
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
        if (state.target) {
            drawTarget();
        }
    }

    function updateProjectileInputs() {
        const presetKey = DOMElements.projectileType.value;
        const preset = PROJECTILE_PRESETS[presetKey];
        if (!preset) return;

        DOMElements.projectileForm.elements['mass'].value = preset.mass;
        DOMElements.projectileForm.elements['dragCoefficient'].value = preset.dragCoefficient;
        DOMElements.projectileForm.elements['referenceArea'].value = preset.referenceArea;
    }

    // --- DATA DISPLAY ---
    function updateLiveSummary(point, maxAltitude, maxSpeed, maxMach = 0) {
        if (!point) return;

        const format = (num) => num.toLocaleString(undefined, { maximumFractionDigits: 1 });

        DOMElements.summary.maxAltitude.textContent = format(maxAltitude);
        DOMElements.summary.range.textContent = format(point.x);
        DOMElements.summary.flightTime.textContent = format(point.time);
        DOMElements.summary.maxSpeed.textContent = format(maxSpeed);

        if (state.simulationType === 'rocket') {
            DOMElements.summary.maxMach.textContent = maxMach.toFixed(2);
            // Burnout altitude is handled by the final updateSummary call
        }
    }


    function updateSummary(result) {
        const data = result.data;
        if (!data || data.length === 0) return;

        const lastPoint = data[data.length - 1] || { x: 0, time: 0 };
        const maxAltitude = Math.max(...data.map(p => p.y));
        const maxSpeed = Math.max(...data.map(p => p.speed));

        const format = (num) => num.toLocaleString(undefined, { maximumFractionDigits: 1 });

        DOMElements.summary.maxAltitude.textContent = format(maxAltitude);
        DOMElements.summary.range.textContent = format(lastPoint.x);
        DOMElements.summary.flightTime.textContent = format(lastPoint.time);
        DOMElements.summary.maxSpeed.textContent = format(maxSpeed);

        if (state.simulationType === 'rocket') {
            const maxMach = Math.max(...data.map(p => p.mach));
            DOMElements.summary.maxMach.textContent = maxMach.toFixed(2);
            DOMElements.summary.burnoutAltitude.textContent = result.burnoutAltitude > 0 ? format(result.burnoutAltitude) : 'N/A';
        }
    }

    function updateCharts(data, isFinalUpdate = false) {
        if (!data) return;

        const gridColor = state.isDarkMode ? 'rgba(255, 255, 255, 0.1)' : 'rgba(0, 0, 0, 0.1)';
        const textColor = state.isDarkMode ? '#f4f7fb' : '#172033';
        Chart.defaults.color = textColor;
        Chart.defaults.borderColor = gridColor;

        const commonOptions = {
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            animation: {
                duration: 0 // Disable default chart animations
            },
            plugins: {
                legend: { position: 'top' },
                zoom: {
                    pan: { 
                        enabled: true, 
                        mode: 'xy',
                        onPanComplete: redrawOverlay
                    },
                    zoom: { 
                        wheel: { enabled: true }, pinch: { enabled: true }, mode: 'xy',
                        onZoomComplete: redrawOverlay
                    }
                }
            },
        };

        // Sync animation canvas size with chart canvas
        DOMElements.animationCanvas.width = DOMElements.trajectoryChartCanvas.clientWidth;
        DOMElements.animationCanvas.height = DOMElements.trajectoryChartCanvas.clientHeight;
        DOMElements.animationCanvas.style.position = 'absolute';
        DOMElements.animationCanvas.style.pointerEvents = 'none';
        if (state.target) drawTarget();

        // Trajectory Chart
        createOrUpdateChart('trajectory', 'trajectory-chart', {
            ...chartConfigs.trajectory,
            options: { ...commonOptions, ...chartConfigs.trajectory.options },
            data: {
                datasets: [{
                    label: 'Trajectory',
                    // On initial call for projectile, data is empty for animation.
                // On final update, populate it fully to ensure it's all there.
                    data: (state.simulationType === 'projectile' && !isFinalUpdate) ? [] : data.map(p => ({ x: p.x, y: p.y })),
                    borderColor: chartColors.blue,
                    backgroundColor: chartColors.blue,
                    showLine: true,
                    pointRadius: 0,
                    tension: 0.1
                }]
            }
        });

        // Velocity Chart
        createOrUpdateChart('velocity', 'velocity-chart', {
            ...chartConfigs.velocity,
            options: { ...commonOptions, ...chartConfigs.velocity.options },
            data: {
                labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                datasets: [
                    {
                        label: 'Total Speed',
                        data: data.map(p => p.speed),
                        borderColor: chartColors.blue,
                        pointRadius: 0,
                        tension: 0.1
                    },
                    {
                        label: 'Vx',
                        data: data.map(p => p.vx),
                        borderColor: chartColors.sky,
                        pointRadius: 0,
                        tension: 0.1,
                        hidden: true
                    },
                    {
                        label: 'Vy',
                        data: data.map(p => p.vy),
                        borderColor: chartColors.green,
                        pointRadius: 0,
                        tension: 0.1,
                        hidden: true
                    }
                ]
            }
        });

        // Altitude Chart
        createOrUpdateChart('altitude', 'altitude-chart', {
            ...chartConfigs.altitude,
            options: { ...commonOptions, ...chartConfigs.altitude.options },
            data: {
                labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                datasets: [{
                    label: 'Altitude',
                    data: data.map(p => p.y),
                    borderColor: chartColors.green,
                    backgroundColor: state.isDarkMode ? 'rgba(22, 121, 79, 0.3)' : 'rgba(22, 121, 79, 0.1)',
                    fill: 'start',
                    pointRadius: 0,
                    tension: 0.1
                }]
            }
        });

        if (state.simulationType === 'rocket') {
            // Mass Chart
            createOrUpdateChart('mass', 'mass-chart', {
                ...chartConfigs.mass,
                options: { ...commonOptions, ...chartConfigs.mass.options },
                data: {
                    labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                    datasets: [{
                        label: 'Mass',
                        data: data.map(p => p.mass),
                        borderColor: chartColors.orange,
                        pointRadius: 0,
                        tension: 0.1
                    }]
                }
            });

            // Mach & Cd Chart
            createOrUpdateChart('mach', 'mach-chart', {
                ...chartConfigs.mach,
                options: {
                    ...commonOptions, ...chartConfigs.mach.options,
                    scales: {
                        x: { title: { display: true, text: 'Time (s)' } },
                        y: { type: 'linear', position: 'left', title: { display: true, text: 'Mach Number' } },
                        y1: { type: 'linear', position: 'right', title: { display: true, text: 'Drag Coeff (Cd)' }, grid: { drawOnChartArea: false } }
                    }
                },
                data: {
                    labels: (isFinalUpdate) ? data.map(p => p.time.toFixed(1)) : [],
                    datasets: [
                        {
                            label: 'Mach',
                            data: data.map(p => p.mach),
                            borderColor: chartColors.red,
                            yAxisID: 'y',
                            pointRadius: 0,
                            tension: 0.1
                        },
                        {
                            label: 'Cd',
                            data: data.map(p => p.cd),
                            borderColor: chartColors.sky,
                            yAxisID: 'y1',
                            pointRadius: 0,
                            tension: 0.1
                        }
                    ]
                }
            });
        }
    }

    function createOrUpdateChart(id, canvasId, config) {
        if (state.charts[id]) {
            state.charts[id].data = config.data;
            state.charts[id].update();
        } else {
            const ctx = document.getElementById(canvasId).getContext('2d');
            state.charts[id] = new Chart(ctx, config);
        }
    }

    function init3DScene() {
        const canvas = DOMElements.threeCanvas;
        const width = canvas.clientWidth;
        const height = canvas.clientHeight;

        // Scene
        three.scene = new THREE.Scene();
        three.scene.background = new THREE.Color(state.isDarkMode ? 0x0f1420 : 0xeef3f8);

        // Camera
        three.camera = new THREE.PerspectiveCamera(60, width / height, 1, 1000000);
        three.camera.position.set(-2000, 2000, 5000); // Start behind and above the origin
        three.camera.lookAt(0, 0, 0); // Look at the artillery station

        // Controls
        three.controls = new OrbitControls(three.camera, canvas);
        three.controls.target.set(0, 0, 0); // Set initial target to the origin
        three.controls.enableDamping = true; // for smooth camera motion
        three.controls.dampingFactor = 0.05;

        // Renderer
        three.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        three.renderer.setSize(width, height);
        three.renderer.setPixelRatio(window.devicePixelRatio);

        // Lighting
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
        three.scene.add(ambientLight);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(-1, 1, 1);
        three.scene.add(directionalLight);

        // Terrain
        const groundSize = 60000;
        const segments = 100;
        const groundGeometry = new THREE.PlaneGeometry(groundSize, groundSize, segments, segments);
        const groundMaterial = new THREE.MeshStandardMaterial({ color: 0x228B22, side: THREE.DoubleSide });
        three.ground = new THREE.Mesh(groundGeometry, groundMaterial);

        // Generate Hills
        const vertices = groundGeometry.attributes.position;
        for (let i = 0; i < vertices.count; i++) {
            const x = vertices.getX(i);
            const y = vertices.getY(i);
            const height = 1500 * (Math.sin(x / 5000) * Math.cos(y / 5000));
            vertices.setZ(i, height); // Displace the Z vertex (which becomes Y after rotation)
        }
        groundGeometry.computeVertexNormals(); // Recalculate normals for correct lighting

        three.ground.rotation.x = -Math.PI / 2;
        // We shift the ground down slightly so the projectile starts just above the average ground level
        three.ground.position.y = -500;
        three.scene.add(three.ground);

        // Add simple landmarks to make the terrain easier to read at a glance.
        three.scenery = new THREE.Group();
        three.scene.add(three.scenery);

        const foliageMaterial = new THREE.MeshStandardMaterial({ color: 0x2f7d4a, roughness: 0.9 });
        const trunkMaterial = new THREE.MeshStandardMaterial({ color: 0x765033, roughness: 1 });
        const wallMaterial = new THREE.MeshStandardMaterial({ color: 0xd9b38c, roughness: 0.85 });
        const roofMaterial = new THREE.MeshStandardMaterial({ color: 0x9b3d32, roughness: 0.8 });
        const targetMaterial = new THREE.MeshStandardMaterial({ color: 0xf2d14b, roughness: 0.7 });
        const targetRingMaterial = new THREE.MeshStandardMaterial({ color: 0xc83e3e, roughness: 0.7 });

        const addForest = (candidateCount = 1800) => {
            const placements = [];
            let seed = 42817;
            const random = () => {
                seed = (seed * 1664525 + 1013904223) >>> 0;
                return seed / 4294967296;
            };

            for (let i = 0; i < candidateCount; i++) {
                const x = THREE.MathUtils.randFloat(-24000, 24000);
                const z = THREE.MathUtils.randFloat(-7000, 7000);
                const clusterShape = 0.5 + 0.5 * Math.sin(x / 3600 + Math.sin(z / 2800)) * Math.cos(z / 3000);
                const density = 0.08 + clusterShape * 0.78;

                if (random() > density) continue;

                placements.push({
                    x,
                    z,
                    scale: THREE.MathUtils.randFloat(0.7, 1.3),
                    rotation: random() * Math.PI * 2,
                });
            }

            const trunkGeometry = new THREE.CylinderGeometry(24, 32, 180, 8);
            const crownGeometry = new THREE.ConeGeometry(125, 300, 8);
            const trunks = new THREE.InstancedMesh(trunkGeometry, trunkMaterial, placements.length);
            const crowns = new THREE.InstancedMesh(crownGeometry, foliageMaterial, placements.length);
            const dummy = new THREE.Object3D();

            placements.forEach(({ x, z, scale, rotation }, index) => {
                const terrainY = getTerrainHeight(x, z);
                dummy.scale.setScalar(scale);
                dummy.rotation.set(0, rotation, 0);

                dummy.position.set(x, terrainY + 90 * scale, z);
                dummy.updateMatrix();
                trunks.setMatrixAt(index, dummy.matrix);

                dummy.position.set(x, terrainY + 290 * scale, z);
                dummy.updateMatrix();
                crowns.setMatrixAt(index, dummy.matrix);
            });

            trunks.instanceMatrix.needsUpdate = true;
            crowns.instanceMatrix.needsUpdate = true;
            trunks.computeBoundingSphere();
            crowns.computeBoundingSphere();
            three.scenery.add(trunks, crowns);
        };

        const addHouse = (x, z, scale = 1) => {
            const house = new THREE.Group();
            const walls = new THREE.Mesh(new THREE.BoxGeometry(420, 260, 360), wallMaterial);
            walls.position.y = 130;
            house.add(walls);

            const roof = new THREE.Mesh(new THREE.ConeGeometry(310, 230, 4), roofMaterial);
            roof.rotation.y = Math.PI / 4;
            roof.position.y = 375;
            house.add(roof);

            house.position.set(x, getTerrainHeight(x, z), z);
            house.scale.setScalar(scale);
            three.scenery.add(house);
        };

        const addTarget = (x, z) => {
            const target = new THREE.Group();
            const pole = new THREE.Mesh(new THREE.CylinderGeometry(14, 14, 520, 8), trunkMaterial);
            pole.position.y = 260;
            target.add(pole);

            const board = new THREE.Mesh(new THREE.CylinderGeometry(170, 170, 24, 32), targetMaterial);
            board.rotation.x = Math.PI / 2;
            board.position.y = 430;
            target.add(board);

            const ring = new THREE.Mesh(new THREE.TorusGeometry(90, 18, 8, 32), targetRingMaterial);
            ring.position.y = 430;
            target.add(ring);

            target.position.set(x, getTerrainHeight(x, z), z);
            three.scenery.add(target);
        };

        addForest(1800);

        // Small settlements sit off the main flight corridor.
        [
            [6200, 1500, 1], [6750, 1650, 0.85], [6900, 1050, 0.75],
            [7500, 1350, 0.9], [7200, 2050, 0.72],
            [15400, -1200, 0.85], [15900, -1000, 0.72], [16100, -1550, 0.8],
            [16600, -1250, 0.68], [-10800, -1700, 0.9], [-10200, -1450, 0.72],
            [-9800, -1950, 0.78]
        ].forEach(([x, z, scale]) => addHouse(x, z, scale));

        addTarget(12000, 0);
        addTarget(24500, 1800);
        addTarget(-16500, -1200);

        // Projectile
        const projectileGeometry = new THREE.SphereGeometry(150, 16, 16);
        const projectileMaterial = new THREE.MeshStandardMaterial({ color: 0xff6347 });
        three.projectile = new THREE.Mesh(projectileGeometry, projectileMaterial);
        three.projectile.visible = true; // Ensure it's visible on new runs
        three.scene.add(three.projectile);
        
        // Artillery Model (Field Gun Style)
        const artillery = new THREE.Group();

        // Materials
        const greenMat = new THREE.MeshStandardMaterial({ color: 0x3b4d28, roughness: 0.7 });
        const darkMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.9 });
        const metalMat = new THREE.MeshStandardMaterial({ color: 0x444444, metalness: 0.6, roughness: 0.3 });

        // 1. Base / Carriage (Sloped trail legs)
        const baseGeom = new THREE.BoxGeometry(0.6, 0.3, 2.0);
        const base = new THREE.Mesh(baseGeom, greenMat);
        base.position.set(0, 0.4, 0.5); // Extended backward along Z-axis
        artillery.add(base);

        // 2. Wheels (Z-axis alignment fix)
        const wheelGeom = new THREE.CylinderGeometry(0.6, 0.6, 0.2, 24);
        wheelGeom.rotateZ(Math.PI / 2); // Rotated around Z so flat faces point out left/right

        const leftWheel = new THREE.Mesh(wheelGeom, darkMat);
        leftWheel.position.set(-0.5, 0.6, 0);
        artillery.add(leftWheel);

        const rightWheel = new THREE.Mesh(wheelGeom, darkMat);
        rightWheel.position.set(0.5, 0.6, 0);
        artillery.add(rightWheel);

        // 3. Axle
        const axleGeom = new THREE.CylinderGeometry(0.08, 0.08, 1.2, 8);
        axleGeom.rotateZ(Math.PI / 2); 
        const axle = new THREE.Mesh(axleGeom, metalMat);
        axle.position.set(0, 0.6, 0);
        artillery.add(axle);

        // 4. Barrel Pivot (Placed directly on the axle line)
        const barrelPivot = new THREE.Group();
        barrelPivot.position.set(0, 0.7, 0); 
        artillery.add(barrelPivot);

        // 5. Barrel (Tapered and rotated to point FORWARD along negative Z-axis)
        // Top radius: 0.1, Bottom radius: 0.18, Length: 2.2
        const barrelGeom = new THREE.CylinderGeometry(0.1, 0.18, 2.2, 16);
        barrelGeom.rotateX(Math.PI / 2); // Crucial: Rotates the vertical cylinder to point along the Z-axis
        const barrel = new THREE.Mesh(barrelGeom, metalMat);

        // Offset the barrel mesh forward so it pivots near the breech (back end)
        barrel.position.set(0, 0, -0.8); 
        barrelPivot.add(barrel);

        // Optional: Give it a slight upward firing angle default (e.g., 15 degrees)
        barrelPivot.rotation.x = -Math.PI / 12; 

        // Scale and position the entire model
        artillery.scale.set(250, 250, 250);
        artillery.position.set(0, -250, 0); 
        three.scene.add(artillery);

        // Trajectory Line
        const lineMaterial = new THREE.LineBasicMaterial({ color: 0x2155cd });
        const lineGeometry = new THREE.BufferGeometry();
        three.line = new THREE.Line(lineGeometry, lineMaterial);
        three.scene.add(three.line);
    }

    function update3DScene(frameData, points) {
        if (!three.renderer) return;

        // Update projectile position
        three.projectile.position.set(frameData.x, frameData.y, 0);

        // Smoothly move the camera's target to follow the projectile
        const targetPosition = new THREE.Vector3(frameData.x, frameData.y, 0);
        three.controls.target.lerp(targetPosition, 0.1);
        three.controls.update();

        // Update trajectory line
        three.line.geometry.setFromPoints(points);

        // Render
        three.renderer.render(three.scene, three.camera);
    }

    /**
     * Creates a particle-based explosion effect at a given position.
     * @param {number} impactX - The world x-coordinate for the explosion.
     * @param {number} impactY - The world y-coordinate for the explosion.
     * @param {number} impactZ - The world z-coordinate for the explosion.
     * @returns {object} A reference to the created particle system and its properties.
     */
    function createBlast(impactX, impactY, impactZ) {
        if (!three.scene) return;
    
        const particleCount = 1000;
        const positions = new Float32Array(particleCount * 3);
        const velocities = [];
    
        // All particles spawn at the exact point of impact
        for (let i = 0; i < particleCount; i++) {
            positions[i * 3] = impactX;
            positions[i * 3 + 1] = impactY;
            positions[i * 3 + 2] = impactZ;
    
            // Spherical blast calculation direction vectors
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.acos((Math.random() * 2) - 1);
            const speed = 50 + Math.random() * 150; // Random force dispersion
    
            velocities.push({
                x: Math.sin(phi) * Math.cos(theta) * speed,
                y: Math.sin(phi) * Math.sin(theta) * speed,
                z: Math.cos(phi) * speed
            });
        }
    
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    
        const material = new THREE.PointsMaterial({
            color: 0xff5500, // Fiery orange blast
            size: 35,
            transparent: true,
            opacity: 1.0,
            blending: THREE.AdditiveBlending, // Glow effect
            depthWrite: false,
        });
    
        const particleSystem = new THREE.Points(geometry, material);
        three.scene.add(particleSystem);
    
        // Return reference along with its unique speed parameters for animation
        return { mesh: particleSystem, geometry, material, velocities, count: particleCount };
    }

    function drawTarget() {
        if (!state.target || !state.charts.trajectory) return;
    
        const chart = state.charts.trajectory;
        const ctx = DOMElements.animationCtx;
        const chartArea = chart.chartArea;
        if (!chartArea) return;
    
        // Convert target data coordinates to pixel coordinates
        const xPixel = chart.scales.x.getPixelForValue(state.target.x);
        const yPixel = chart.scales.y.getPixelForValue(state.target.y);
    
        // Convert 100-unit radius to pixels. Average of X and Y scales for a circular appearance.
        const xPixelRadius = chart.scales.x.getPixelForValue(state.target.x + 100) - xPixel;
        const yPixelRadius = yPixel - chart.scales.y.getPixelForValue(state.target.y + 100);
        const pixelRadius = (xPixelRadius + yPixelRadius) / 2;
    
        // Draw hitbox circle
        ctx.beginPath();
        ctx.arc(xPixel, yPixel, pixelRadius, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(255, 99, 132, 0.3)'; // Semi-transparent red
        ctx.strokeStyle = 'rgba(255, 99, 132, 0.8)';
        ctx.lineWidth = 2;
        ctx.fill();
        ctx.stroke();
    }

    function drawAnimatedProjectile(position, chart) {
        const ctx = DOMElements.animationCtx;
        const chartArea = chart.chartArea;
        if (!chartArea) return;

        // Convert data coordinates to canvas pixel coordinates
        const xPixel = chart.scales.x.getPixelForValue(position.x);
        const yPixel = chart.scales.y.getPixelForValue(position.y);

        // Clear previous frame
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);

        // Redraw the target so the projectile appears on top of it
        if (state.target) drawTarget();

        // Draw projectile if it's within the chart area
        if (xPixel >= chartArea.left && xPixel <= chartArea.right && yPixel >= chartArea.top && yPixel <= chartArea.bottom) {
            ctx.beginPath();
            ctx.arc(xPixel, yPixel, 5, 0, 2 * Math.PI);
            ctx.fillStyle = state.isDarkMode ? chartColors.orange : chartColors.red;
            ctx.fill();
        }
    }

    function updateTable(data) {
        if (!data) {
            resetUI();
            return;
        }

        updateTableHeaders();

        const limit = parseInt(DOMElements.table.limit.value, 10);
        const step = Math.max(1, Math.floor(data.length / limit));
        const sampledData = data.filter((_, i) => i % step === 0 || i === data.length - 1).slice(0, limit);

        const format = (num) => num.toLocaleString(undefined, { minimumFractionDigits: 2, maximumFractionDigits: 2 });

        let tableHTML = '';
        if (state.simulationType === 'projectile') {
            tableHTML = sampledData.map(p => `
                <tr>
                    <td>${format(p.time)}</td>
                    <td>${format(p.x)}</td>
                    <td>${format(p.y)}</td>
                    <td>${format(p.vx)}</td>
                    <td>${format(p.vy)}</td>
                    <td>${format(p.speed)}</td>
                </tr>
            `).join('');
        } else { // Rocket
            tableHTML = sampledData.map(p => `
                <tr>
                    <td>${format(p.time)}</td>
                    <td>${format(p.x)}</td>
                    <td>${format(p.y)}</td>
                    <td>${format(p.speed)}</td>
                    <td>${p.mach.toFixed(3)}</td>
                    <td>${p.cd.toFixed(3)}</td>
                    <td>${format(p.mass)}</td>
                </tr>
            `).join('');
        }

        DOMElements.table.body.innerHTML = tableHTML;
    }

    function updateTableHeaders() {
        let headers = [];
        if (state.simulationType === 'projectile') {
            headers = ['Time (s)', 'X (m)', 'Y (m)', 'Vx (m/s)', 'Vy (m/s)', 'Speed (m/s)'];
        } else {
            headers = ['Time (s)', 'X (m)', 'Y (m)', 'Speed (m/s)', 'Mach', 'Cd', 'Mass (kg)'];
        }
        DOMElements.table.head.innerHTML = headers.map(h => `<th scope="col">${h}</th>`).join('');
    }

    // --- EXPORT ---
    function exportToCSV() {
        if (!state.simulationData) return;

        const data = state.simulationData;
        const headers = Object.keys(data[0]);
        const csvRows = [
            headers.join(','),
            ...data.map(row => headers.map(header => row[header]).join(','))
        ];

        const csvString = csvRows.join('\n');
        const blob = new Blob([csvString], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${state.simulationType}_trajectory_${new Date().toISOString()}.csv`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    // --- START THE APP ---
    init();
});
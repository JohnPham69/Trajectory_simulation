/* =========================================================
   FLIGHT TRAJECTORY SIMULATOR
   Application and Simulation Logic
   ========================================================= */

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
        charts: {},
        isDarkMode: window.matchMedia('(prefers-color-scheme: dark)').matches
    };

    // --- DOM ELEMENTS ---
    const DOMElements = {
        themeToggle: document.getElementById('theme-toggle'),
        projectileTab: document.getElementById('projectile-tab'),
        rocketTab: document.getElementById('rocket-tab'),
        projectilePanel: document.getElementById('projectile-panel'),
        rocketPanel: document.getElementById('rocket-panel'),
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
        chartResetButtons: document.querySelectorAll('.chart-reset-button'),
    };

    // --- CHART CONFIGURATION ---
    const chartConfigs = {
        trajectory: {
            type: 'scatter',
            options: { scales: { x: { title: { display: true, text: 'Range (m)' } }, y: { title: { display: true, text: 'Altitude (m)' } } } }
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
        resetUI();
    }

    // --- EVENT LISTENERS ---
    function setupEventListeners() {
        DOMElements.themeToggle.addEventListener('click', toggleTheme);
        DOMElements.projectileTab.addEventListener('click', () => switchSimulationType('projectile'));
        DOMElements.rocketTab.addEventListener('click', () => switchSimulationType('rocket'));
        DOMElements.runButton.addEventListener('click', runSimulation);
        DOMElements.resetButton.addEventListener('click', resetUI);
        DOMElements.exportButton.addEventListener('click', exportToCSV);
        DOMElements.table.limit.addEventListener('change', () => updateTable(state.simulationData));
        DOMElements.chartResetButtons.forEach(button => {
            button.addEventListener('click', () => {
                const chartId = button.dataset.chart;
                if (state.charts[chartId]) {
                    state.charts[chartId].resetZoom();
                }
            });
        });
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
            updateCharts(state.simulationData);
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
        const form = state.simulationType === 'projectile' ? DOMElements.projectileForm : DOMElements.rocketForm;
        const params = getAndValidateFormParams(form);

        if (!params) return;

        setUIState('running');

        // Use setTimeout to allow the UI to update before the heavy computation
        setTimeout(() => {
            try {
                let result;
                if (state.simulationType === 'projectile') {
                    result = simulateProjectileTrajectory(params);
                } else {
                    result = simulateV2Trajectory(params);
                }
                state.simulationData = result.data;
                
                updateSummary(result);
                updateCharts(result.data);
                updateTable(result.data);

                setUIState('complete');
            } catch (error) {
                console.error("Simulation failed:", error);
                setUIState('error', 'Simulation failed. Check console for details.');
            }
        }, 50);
    }

    function getAndValidateFormParams(form) {
        const formData = new FormData(form);
        const params = {};
        let isValid = true;
        let errorMessage = '';

        for (const [name, value] of formData.entries()) {
            const input = form.elements[name];
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
        const isRunning = status === 'running';
        DOMElements.runButton.disabled = isRunning;
        DOMElements.resetButton.disabled = isRunning;
        DOMElements.exportButton.disabled = isRunning || !state.simulationData;

        DOMElements.statusBadge.classList.remove('success', 'error', 'running');
        switch (status) {
            case 'running':
                DOMElements.statusBadge.textContent = 'Running...';
                DOMElements.statusBadge.classList.add('running');
                break;
            case 'complete':
                DOMElements.statusBadge.textContent = 'Complete';
                DOMElements.statusBadge.classList.add('success');
                break;
            case 'error':
                DOMElements.statusBadge.textContent = 'Error';
                DOMElements.statusBadge.classList.add('error');
                DOMElements.formError.textContent = message;
                DOMElements.formError.hidden = false;
                break;
            default: // 'ready'
                DOMElements.statusBadge.textContent = 'Ready';
                break;
        }
    }

    function resetUI() {
        state.simulationData = null;
        
        // Reset summary cards
        Object.values(DOMElements.summary).forEach(el => el.textContent = '—');

        // Clear charts
        Object.values(state.charts).forEach(chart => chart.destroy());
        state.charts = {};

        // Clear table
        DOMElements.table.body.innerHTML = `<tr><td colspan="8" class="empty-table-message">Run a simulation to view calculated data.</td></tr>`;
        updateTableHeaders();

        // Reset forms (optional, could be annoying)
        // DOMElements.projectileForm.reset();
        // DOMElements.rocketForm.reset();

        DOMElements.formError.hidden = true;
        setUIState('ready');
    }

    // --- DATA DISPLAY ---
    function updateSummary(result) {
        const data = result.data;
        if (!data || data.length === 0) return;

        const lastPoint = data[data.length - 1];
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

    function updateCharts(data) {
        if (!data) return;

        const gridColor = state.isDarkMode ? 'rgba(255, 255, 255, 0.1)' : 'rgba(0, 0, 0, 0.1)';
        const textColor = state.isDarkMode ? '#f4f7fb' : '#172033';
        Chart.defaults.color = textColor;
        Chart.defaults.borderColor = gridColor;

        const commonOptions = {
            responsive: true,
            maintainAspectRatio: false,
            interaction: { mode: 'index', intersect: false },
            plugins: {
                legend: { position: 'top' },
                zoom: {
                    pan: { enabled: true, mode: 'xy' },
                    zoom: { wheel: { enabled: true }, pinch: { enabled: true }, mode: 'xy' }
                }
            },
        };

        // Trajectory Chart
        createOrUpdateChart('trajectory', 'trajectory-chart', {
            ...chartConfigs.trajectory,
            options: { ...commonOptions, ...chartConfigs.trajectory.options },
            data: {
                datasets: [{
                    label: 'Trajectory',
                    data: data.map(p => ({ x: p.x, y: p.y })),
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
                labels: data.map(p => p.time.toFixed(1)),
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
                labels: data.map(p => p.time.toFixed(1)),
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
                    labels: data.map(p => p.time.toFixed(1)),
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
                    labels: data.map(p => p.time.toFixed(1)),
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
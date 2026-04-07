// ARMageddon Control UI - Client Application

(function () {
    "use strict";

    // --- WebSocket ---
    let ws = null;
    let state = {
        power: false,
        mode: "joint",
        axis_index: 0,
        sensitivity_index: 3,
        jog_position: 0,
        active_axis: "J1",
        axes: ["J1", "J2", "J3", "J4", "J5", "J6"],
        sensitivity_pct: 50,
        jog_step_size: 2.5,
        sensitivity_detents: [5, 10, 25, 50, 75, 100],
        sensitivity_count: 6,
        base_jog_step: 5.0,
    };

    function connect() {
        const proto = location.protocol === "https:" ? "wss:" : "ws:";
        ws = new WebSocket(`${proto}//${location.host}/ws`);

        ws.onopen = () => {
            document.getElementById("connection-status").textContent = "CONNECTED";
            document.getElementById("connection-status").className = "connected";
        };

        ws.onclose = () => {
            document.getElementById("connection-status").textContent = "DISCONNECTED";
            document.getElementById("connection-status").className = "disconnected";
            setTimeout(connect, 2000);
        };

        ws.onerror = () => ws.close();

        ws.onmessage = (evt) => {
            const msg = JSON.parse(evt.data);
            if (msg.type === "state") {
                state = msg;
                render();
            }
        };
    }

    function send(event) {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify(event));
        }
    }

    // --- Audio feedback for detent clicks ---
    const audioCtx = new (window.AudioContext || window.webkitAudioContext)();

    function playDetentClick() {
        const osc = audioCtx.createOscillator();
        const gain = audioCtx.createGain();
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.frequency.value = 800;
        osc.type = "square";
        gain.gain.value = 0.05;
        gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + 0.04);
        osc.start();
        osc.stop(audioCtx.currentTime + 0.04);
    }

    // --- Detent Dots SVG Rendering ---

    function renderDetentDots(svgId, count, options) {
        const svg = document.getElementById(svgId);
        svg.innerHTML = "";
        const cx = 55, cy = 55, r = 52; // center and radius of dot ring
        const startAngle = options.startDeg || -135;
        const endAngle = options.endDeg || 135;
        const totalArc = endAngle - startAngle;
        const activeIndex = options.activeIndex;

        for (let i = 0; i < count; i++) {
            const frac = count === 1 ? 0.5 : i / (count - 1);
            const angleDeg = startAngle + frac * totalArc;
            const angleRad = (angleDeg - 90) * (Math.PI / 180); // -90 so 0° = top
            const dx = cx + r * Math.cos(angleRad);
            const dy = cy + r * Math.sin(angleRad);

            const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
            circle.setAttribute("cx", dx.toFixed(1));
            circle.setAttribute("cy", dy.toFixed(1));
            circle.setAttribute("r", i === activeIndex ? "4" : "2.5");
            circle.setAttribute("fill", i === activeIndex ? (options.activeColor || "#2196f3") : "#555");
            svg.appendChild(circle);

            // Label for sensitivity detents
            if (options.labels && options.labels[i]) {
                const text = document.createElementNS("http://www.w3.org/2000/svg", "text");
                const labelR = r + 12;
                const lx = cx + labelR * Math.cos(angleRad);
                const ly = cy + labelR * Math.sin(angleRad);
                text.setAttribute("x", lx.toFixed(1));
                text.setAttribute("y", (ly + 3).toFixed(1));
                text.setAttribute("text-anchor", "middle");
                text.setAttribute("fill", i === activeIndex ? (options.activeColor || "#2196f3") : "#666");
                text.setAttribute("font-size", "7");
                text.setAttribute("font-weight", "700");
                text.setAttribute("font-family", "'Courier New', monospace");
                text.textContent = options.labels[i];
                svg.appendChild(text);
            }
        }
    }

    // Jog knob: 18 evenly spaced detent dots around full circle (like encoder notches)
    const JOG_DETENT_COUNT = 18;

    function renderJogDots() {
        const svg = document.getElementById("jog-detent-ring");
        svg.innerHTML = "";
        const cx = 55, cy = 55, r = 52;

        for (let i = 0; i < JOG_DETENT_COUNT; i++) {
            const angleDeg = (i / JOG_DETENT_COUNT) * 360;
            const angleRad = (angleDeg - 90) * (Math.PI / 180);
            const dx = cx + r * Math.cos(angleRad);
            const dy = cy + r * Math.sin(angleRad);

            const circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
            circle.setAttribute("cx", dx.toFixed(1));
            circle.setAttribute("cy", dy.toFixed(1));
            circle.setAttribute("r", "2");
            circle.setAttribute("fill", "#555");
            svg.appendChild(circle);
        }
    }

    // --- Rendering ---
    function render() {
        const enclosure = document.querySelector(".enclosure");
        const powerSwitch = document.getElementById("power-switch");

        // Power
        if (state.power) {
            powerSwitch.classList.add("on");
            powerSwitch.classList.remove("off");
            enclosure.classList.remove("disabled");
        } else {
            powerSwitch.classList.remove("on");
            powerSwitch.classList.add("off");
            enclosure.classList.add("disabled");
        }

        // Power status
        const statusPower = document.getElementById("status-power");
        statusPower.textContent = state.power ? "ON" : "OFF";
        statusPower.className = "status-indicator " + (state.power ? "on" : "off");

        // Mode toggle thumb
        const thumb = document.querySelector(".toggle-thumb");
        thumb.className = "toggle-thumb " + state.mode;

        // Mode toggle labels
        document.querySelectorAll(".toggle-labels span").forEach((el) => {
            el.classList.toggle("active", el.dataset.mode === state.mode);
        });

        // Mode badge
        const modeBadge = document.getElementById("status-mode");
        modeBadge.textContent = state.mode.toUpperCase();
        modeBadge.className = "mode-badge " + state.mode;

        // Active axis
        const axisValue = document.querySelector("#status-axis .axis-value");
        axisValue.textContent = state.active_axis;

        // Axis map
        const axisMap = document.getElementById("axis-map");
        axisMap.innerHTML = "";
        if (state.mode === "leader") {
            const chip = document.createElement("span");
            chip.className = "axis-chip active leader";
            chip.textContent = "ALL (1:1)";
            axisMap.appendChild(chip);
        } else {
            (state.axes || []).forEach((axis, i) => {
                const chip = document.createElement("span");
                chip.className = "axis-chip";
                if (i === state.axis_index) {
                    chip.classList.add("active", state.mode);
                }
                chip.textContent = axis;
                axisMap.appendChild(chip);
            });
        }

        // Sensitivity knob rotation — snap to detent positions
        const sensCount = state.sensitivity_count || 6;
        const sensAngle = -135 + (state.sensitivity_index / (sensCount - 1)) * 270;
        const sensIndicator = document.querySelector("#sensitivity-knob .knob-indicator");
        sensIndicator.style.transform = `translateX(-50%) rotate(${sensAngle}deg)`;

        // Sensitivity detent dots (with labels)
        const detentLabels = (state.sensitivity_detents || []).map((d) => d + "%");
        renderDetentDots("sens-detent-ring", sensCount, {
            startDeg: -135,
            endDeg: 135,
            activeIndex: state.sensitivity_index,
            activeColor: "#2196f3",
            labels: detentLabels,
        });

        // Sensitivity readout under knob
        document.getElementById("sensitivity-readout").textContent = state.sensitivity_pct + "%";

        // Status panel: sensitivity %
        document.querySelector("#sensitivity-pct-display .pct-value").textContent =
            state.sensitivity_pct + "%";

        // Status panel: detent chips
        const detentsContainer = document.getElementById("sensitivity-detents");
        detentsContainer.innerHTML = "";
        (state.sensitivity_detents || []).forEach((pct, i) => {
            const chip = document.createElement("span");
            chip.className = "detent-chip";
            if (i === state.sensitivity_index) chip.classList.add("active");
            chip.textContent = pct + "%";
            detentsContainer.appendChild(chip);
        });

        // Status panel: effective jog step
        document.querySelector("#jog-step-display .step-value").textContent =
            state.jog_step_size.toFixed(2) + "°";

        // Jog position
        document.querySelector("#status-jog .jog-value").textContent =
            state.jog_position.toFixed(2);

        // Jog knob disabled state
        const jogKnob = document.getElementById("jog-knob");
        if (state.mode === "leader" || !state.power) {
            jogKnob.classList.add("disabled");
        } else {
            jogKnob.classList.remove("disabled");
        }

        // Scroll buttons disabled in leader mode
        document.getElementById("scroll-up").disabled = state.mode === "leader";
        document.getElementById("scroll-down").disabled = state.mode === "leader";
    }

    // --- Event Handlers ---

    // Power switch
    document.getElementById("power-switch").addEventListener("click", () => {
        send({ action: "power_toggle" });
    });

    document.getElementById("power-switch").addEventListener("keydown", (e) => {
        if (e.key === "Enter" || e.key === " ") {
            e.preventDefault();
            send({ action: "power_toggle" });
        }
    });

    // Mode toggle
    document.getElementById("mode-toggle").addEventListener("click", () => {
        if (state.power) send({ action: "mode_cycle" });
    });

    // Scroll buttons
    document.getElementById("scroll-up").addEventListener("click", () => {
        if (state.power) send({ action: "scroll_up" });
    });

    document.getElementById("scroll-down").addEventListener("click", () => {
        if (state.power) send({ action: "scroll_down" });
    });

    // --- Detented Knob Interaction ---

    function setupDetentKnob(elementId, options) {
        const el = document.getElementById(elementId);
        const DETENT_THRESHOLD = options.detentDegrees || 30;
        let dragging = false;
        let lastAngle = null;
        let accumulated = 0;

        function getAngle(e, rect) {
            const cx = rect.left + rect.width / 2;
            const cy = rect.top + rect.height / 2;
            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;
            return Math.atan2(clientY - cy, clientX - cx) * (180 / Math.PI);
        }

        function onStart(e) {
            if (el.classList.contains("disabled")) return;
            if (!state.power) return;
            if (audioCtx.state === "suspended") audioCtx.resume();
            dragging = true;
            lastAngle = getAngle(e, el.getBoundingClientRect());
            accumulated = 0;
            e.preventDefault();
        }

        function onMove(e) {
            if (!dragging) return;
            const rect = el.getBoundingClientRect();
            const angle = getAngle(e, rect);
            let delta = angle - lastAngle;

            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;

            lastAngle = angle;
            accumulated += delta;

            while (accumulated >= DETENT_THRESHOLD) {
                accumulated -= DETENT_THRESHOLD;
                playDetentClick();
                options.onClick(+1);
            }
            while (accumulated <= -DETENT_THRESHOLD) {
                accumulated += DETENT_THRESHOLD;
                playDetentClick();
                options.onClick(-1);
            }

            e.preventDefault();
        }

        function onEnd() {
            dragging = false;
            lastAngle = null;
            accumulated = 0;
        }

        el.addEventListener("mousedown", onStart);
        el.addEventListener("touchstart", onStart, { passive: false });
        window.addEventListener("mousemove", onMove);
        window.addEventListener("touchmove", onMove, { passive: false });
        window.addEventListener("mouseup", onEnd);
        window.addEventListener("touchend", onEnd);

        // Scroll wheel support
        el.addEventListener("wheel", (e) => {
            if (el.classList.contains("disabled")) return;
            if (!state.power) return;
            e.preventDefault();
            if (audioCtx.state === "suspended") audioCtx.resume();
            const dir = e.deltaY > 0 ? -1 : 1;
            playDetentClick();
            options.onClick(dir);
        }, { passive: false });
    }

    // Jog knob — each detent click = one jog step (base_step * sensitivity%)
    let jogVisualAngle = 0;
    setupDetentKnob("jog-knob", {
        detentDegrees: 20,
        onClick(direction) {
            jogVisualAngle += direction * 20;
            const indicator = document.querySelector("#jog-knob .knob-indicator");
            indicator.style.transform = `translateX(-50%) rotate(${jogVisualAngle}deg)`;
            send({ action: "jog_click", direction: direction });
        },
    });

    // Sensitivity knob — each detent click = step up/down through % detents
    setupDetentKnob("sensitivity-knob", {
        detentDegrees: 30,
        onClick(direction) {
            send({ action: direction > 0 ? "sensitivity_up" : "sensitivity_down" });
        },
    });

    // --- Keyboard shortcuts ---
    document.addEventListener("keydown", (e) => {
        if (!state.power && e.key !== "p") return;

        switch (e.key) {
            case "p":
                send({ action: "power_toggle" });
                break;
            case "m":
                send({ action: "mode_cycle" });
                break;
            case "ArrowUp":
                e.preventDefault();
                send({ action: "scroll_up" });
                break;
            case "ArrowDown":
                e.preventDefault();
                send({ action: "scroll_down" });
                break;
            case "ArrowRight":
                if (state.mode !== "leader") {
                    playDetentClick();
                    send({ action: "jog_click", direction: 1 });
                }
                break;
            case "ArrowLeft":
                if (state.mode !== "leader") {
                    playDetentClick();
                    send({ action: "jog_click", direction: -1 });
                }
                break;
            case "+":
            case "=":
                playDetentClick();
                send({ action: "sensitivity_up" });
                break;
            case "-":
            case "_":
                playDetentClick();
                send({ action: "sensitivity_down" });
                break;
        }
    });

    // --- Init ---
    renderJogDots();
    render();
    connect();
})();

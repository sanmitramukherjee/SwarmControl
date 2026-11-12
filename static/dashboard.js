// ======================== Configuration Constants ========================
    const WS_URL                   = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.hostname}:8001`;
    const WS_INITIAL_RETRY_DELAY   = 1000;   // ms – first reconnect delay
    const WS_MAX_DELAY             = 30000;  // ms – cap on exponential backoff
    const MAX_LOG_ENTRIES          = 150;    // keep last N log lines in sidebar
    const DRONE_LIST_UPDATE_MS     = 200;    // throttle drone-list DOM updates (ms)
    // 3D scene distance (metres) that triggers the collision warning
    const COLLISION_WARN_M         = 1.5;
    // Three.js hex colours for drone body material
    const DRONE_NORMAL_COLOR       = 0x55d6be;
    const DRONE_COLLISION_COLOR    = 0xff2222;
    const DRONE_NORMAL_EMISSIVE    = 0x123e3a;
    const DRONE_COLLISION_EMISSIVE = 0x550000;
    let scene, camera, renderer, controls, ground, grid;
    let trackerRenderer, trackerCamera;

    // ======================== Dashboard theme ========================
    const themeToggle = document.getElementById('theme-toggle');
    function applyTheme(theme) {
      const dark = theme === 'dark';
      document.documentElement.dataset.theme = dark ? 'dark' : 'light';
      document.querySelector('meta[name="theme-color"]').content = dark ? '#111a22' : '#f2f4f7';
      if (scene) {
        scene.background.set(getComputedStyle(document.documentElement).getPropertyValue('--canvas-bg').trim());
        scene.fog.color.set(getComputedStyle(document.documentElement).getPropertyValue('--canvas-fog').trim());
        if (ground) ground.material.color.set(getComputedStyle(document.documentElement).getPropertyValue('--canvas-bg').trim());
        if (grid) {
          grid.material[0].color.set(getComputedStyle(document.documentElement).getPropertyValue('--canvas-grid').trim());
          grid.material[1].color.set(getComputedStyle(document.documentElement).getPropertyValue('--canvas-grid-secondary').trim());
        }
      }
      if (themeToggle) {
        themeToggle.textContent = dark ? 'Light mode' : 'Dark mode';
        themeToggle.setAttribute('aria-label', `Switch to ${dark ? 'light' : 'dark'} theme`);
        themeToggle.setAttribute('aria-pressed', String(dark));
      }
    }
    let savedTheme = 'light';
    try { savedTheme = localStorage.getItem('swarmcontrol-theme') || 'light'; } catch (_) { /* Storage can be disabled. */ }
    applyTheme(savedTheme);
    if (themeToggle) themeToggle.addEventListener('click', () => {
      const theme = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
      applyTheme(theme);
      try { localStorage.setItem('swarmcontrol-theme', theme); } catch (_) { /* Theme remains active for this page. */ }
    });

    // Keep the event log truly compact when collapsed, including on small screens.
    const sidebarToggle = document.getElementById('sidebar-toggle');
    function updateSidebarToggle() {
      if (!sidebarToggle) return;
      const collapsed = document.getElementById('sidebar').classList.contains('collapsed');
      sidebarToggle.setAttribute('aria-expanded', String(!collapsed));
      sidebarToggle.setAttribute('aria-label', `${collapsed ? 'Expand' : 'Collapse'} system event log`);
      sidebarToggle.title = `${collapsed ? 'Expand' : 'Collapse'} event log`;
    }
    const sidebarObserver = new MutationObserver(updateSidebarToggle);
    sidebarObserver.observe(document.getElementById('sidebar'), { attributes: true, attributeFilter: ['class'] });
    updateSidebarToggle();

    // ======================== Panel Toggle ========================
    function togglePanel(bodyId, arrowId) {
      const body = document.getElementById(bodyId);
      const arrow = document.getElementById(arrowId);
      const collapsed = body.classList.toggle('collapsed');
      arrow.textContent = collapsed ? '▼' : '▲';
      const trigger = document.querySelector(`button[aria-controls="${bodyId}"]`);
      if (trigger) trigger.setAttribute('aria-expanded', String(!collapsed));
    }

    // ======================== Sidebar Toggle ========================
    function toggleSidebar(evt) {
      if (evt) evt.stopPropagation();
      const sidebar = document.getElementById('sidebar');
      const collapsed = sidebar.classList.toggle('collapsed');
      const button = document.getElementById('sidebar-toggle');
      if (button) button.setAttribute('aria-expanded', String(!collapsed));
    }

    const _sidebarToggle = document.getElementById('sidebar-toggle');
    if (_sidebarToggle) _sidebarToggle.addEventListener('click', toggleSidebar);

    // ======================== WebSocket with Exponential Backoff ========================
    let ws = null;
    let wsRetryDelay = WS_INITIAL_RETRY_DELAY;
    document.getElementById('sb-mode-val').textContent = '—';
    document.getElementById('sb-leader-val').textContent = '—';
    document.getElementById('sb-drones-val').textContent = '—';
    document.getElementById('active-count').textContent = '—';
    document.getElementById('avg-alt').textContent = '—';

    function connectWS() {
      ws = new WebSocket(WS_URL);

      ws.onopen = () => {
        wsRetryDelay = WS_INITIAL_RETRY_DELAY;
        document.getElementById('sb-ws-val').textContent = 'CONNECTED';
        document.getElementById('sb-ws-val').closest('.sb-item').classList.add('good');
        document.getElementById('sb-ws-val').closest('.sb-item').classList.remove('warn', 'danger');
        document.getElementById('reconnect-banner').style.display = 'none';
        document.getElementById('loading').style.display = 'none';
        log('WebSocket connected.', 'sys');
      };

      ws.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data);
          if (msg.type === 'log') {
            log(msg.content, 'info');
          } else if (msg.type === 'telemetry') {
            pendingTelemetry = Object.assign(pendingTelemetry || {}, msg.data);
          } else if (msg.type === 'status') {
            applyStatus(msg.data);
          }
        } catch (e) {
          console.error('[WS] Parse error:', e);
        }
      };

      ws.onclose = () => {
        document.getElementById('sb-ws-val').textContent = 'DISCONNECTED';
        document.getElementById('sb-ws-val').closest('.sb-item').classList.add('danger');
        document.getElementById('sb-ws-val').closest('.sb-item').classList.remove('good', 'warn');
        document.getElementById('reconnect-banner').style.display = 'block';
        log(`WebSocket closed. Retrying in ${(wsRetryDelay/1000).toFixed(1)}s…`, 'error');
        setTimeout(() => {
          wsRetryDelay = Math.min(wsRetryDelay * 2, WS_MAX_DELAY);
          connectWS();
        }, wsRetryDelay);
      };

      ws.onerror = () => {
        document.getElementById('sb-ws-val').textContent = 'ERROR';
        document.getElementById('sb-ws-val').closest('.sb-item').classList.add('danger');
        document.getElementById('sb-ws-val').closest('.sb-item').classList.remove('good', 'warn');
        ws.close();
      };
    }

    connectWS();

    // ======================== Telemetry Batching ========================
    // pendingTelemetry accumulates updates between animation frames
    let pendingTelemetry = null;

    // ======================== Log ========================
    const logContainer = document.getElementById('log-container');
    function log(text, type = 'info') {
      const div = document.createElement('div');
      div.className = `log-entry ${type}`;
      div.textContent = `> ${text}`;
      logContainer.appendChild(div);
      if (logContainer.children.length > MAX_LOG_ENTRIES) logContainer.removeChild(logContainer.firstChild);
      logContainer.scrollTop = logContainer.scrollHeight;
    }

    // ======================== Status Sync ========================
    let currentLeaderSysid = null;

    function applyStatus(data) {
      if (data.mode !== undefined && data.mode !== null && String(data.mode).trim()) {
        document.getElementById('sb-mode-val').textContent = String(data.mode).toUpperCase();
      }
      if (data.leader !== undefined && data.leader !== null && Number.isFinite(Number(data.leader))) {
        currentLeaderSysid = Number(data.leader);
        const label = currentLeaderSysid === -1 ? 'AUTO' : `#${currentLeaderSysid}`;
        document.getElementById('sb-leader-val').textContent = label;
        syncLeaderSelect();
      }
      if (data.drone_count !== undefined) {
        document.getElementById('sb-drones-val').textContent = Number.isFinite(Number(data.drone_count))
          ? String(data.drone_count) : '—';
      }
      if (data.manual_mode !== undefined) {
        isManualModeActive = data.manual_mode === true || data.manual_mode === 1 || data.manual_mode === 'true';
        manualModeKnown = true;
        updateManualModeUI();
      }
    }

    let isManualModeActive = false;
    let manualModeKnown = false;

    function toggleManualMode() {
      const fState = getFormationState();
      const payload = {
        type: 'swarm_command',
        cmd: 'manual_mode',
        ...fState,
        alt: readFiniteInput('cmd-alt'),
      };
      if (wsSend(payload)) log('Manual mode command sent.', 'sys');
    }

    function updateManualModeUI() {
      const btn = document.getElementById('btn-manual-mode');
      if (!btn) return;
      if (!manualModeKnown) {
        btn.classList.remove('active');
        btn.textContent = 'MANUAL MODE: STATUS UNKNOWN';
        return;
      }
      if (isManualModeActive) {
        btn.classList.add('active');
        btn.textContent = 'MANUAL MODE: ACTIVE (CLICK TO STOP)';
      } else {
        btn.classList.remove('active');
        btn.textContent = 'MANUAL MODE (RC FOLLOW)';
      }
    }
    updateManualModeUI();

    // ======================== Leader Control ========================
    function onLeaderChange(val) {
      if (val === '') return;
      const sysid = Number(val);
      if (!Number.isInteger(sysid)) return;
      const label = sysid === -1 ? 'AUTO' : `#${sysid}`;
      if (!wsSend({ type: 'set_leader', sysid })) return;
      currentLeaderSysid = sysid;
      document.getElementById('sb-leader-val').textContent = label;
      log(`Leader set to ${label}`, 'sys');
    }

    function syncLeaderSelect() {
      const sel = document.getElementById('leader-select');
      if (!sel) return;
      if (currentLeaderSysid === null) {
        let waiting = [...sel.options].find(option => option.value === '');
        if (!waiting) {
          waiting = document.createElement('option');
          waiting.value = '';
          waiting.textContent = 'Awaiting leader status';
          waiting.disabled = true;
          sel.insertBefore(waiting, sel.firstChild);
        }
        sel.value = '';
        return;
      }
      const value = String(currentLeaderSysid);
      if (![...sel.options].some(option => option.value === value) && currentLeaderSysid !== -1) {
        const option = document.createElement('option');
        option.value = value;
        option.textContent = `DRONE #${value}`;
        sel.appendChild(option);
      }
      sel.value = value;
    }

    function updateLeaderSelect() {
      const sel = document.getElementById('leader-select');
      const existing = new Set([...sel.options].map(o => o.value));
      const sortedIds = Object.keys(drones).sort((a, b) => parseInt(a) - parseInt(b));
      sortedIds.forEach(id => {
        if (!existing.has(id)) {
          const opt = document.createElement('option');
          opt.value = id;
          opt.textContent = `DRONE #${id}`;
          sel.appendChild(opt);
        }
      });
      syncLeaderSelect();

      // Ensure formation indicator reflects the default V-shape on load
      const fType = document.getElementById('formation-type').value;
      document.getElementById('sb-formation-val').textContent = fType !== 'off' ? fType : 'OFF';
    }

    // ======================== Formation Control ========================
    function onFormationTypeChange(val) {
      const isOn = val !== 'off';
      document.getElementById('sb-formation-val').textContent = isOn ? val : 'OFF';
    }

    function getFormationState() {
      const fType = document.getElementById('formation-type').value;
      return {
        formation: fType !== 'off',
        pattern: fType === 'off' ? 'V' : fType,
        spacing_m: parseFloat(document.getElementById('spacing-slider').value),
      };
    }

    // ======================== Swarm Commands ========================
    function wsSend(payload) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        try {
          ws.send(JSON.stringify(payload));
          const feedback = document.getElementById('command-feedback');
          if (feedback) feedback.textContent = 'Command transmitted over the dashboard connection.';
          return true;
        } catch (error) {
          const message = `WebSocket send failed: ${error.message || error}`;
          log(message, 'error');
          const feedback = document.getElementById('command-feedback');
          if (feedback) feedback.textContent = `Command not sent: ${message}`;
          return false;
        }
      }
      log('WebSocket not connected – command dropped.', 'error');
      const feedback = document.getElementById('command-feedback');
      if (feedback) feedback.textContent = 'Command not sent: WebSocket is disconnected.';
      return false;
    }

    function readFiniteInput(id) {
      const raw = document.getElementById(id).value.trim();
      if (!raw) return null;
      const value = Number(raw);
      return Number.isFinite(value) ? value : null;
    }

    function swarmCmd(cmd) {
      const fState = getFormationState();
      const payload = {
        type: 'swarm_command',
        cmd,
        ...fState,
      };
      if (cmd === 'fly_to') {
        const lat = readFiniteInput('cmd-lat');
        const lon = readFiniteInput('cmd-lon');
        const alt = readFiniteInput('cmd-alt');
        if (lat === null || lat < -90 || lat > 90) return commandError('Latitude must be between -90 and 90.');
        if (lon === null || lon < -180 || lon > 180) return commandError('Longitude must be between -180 and 180.');
        if (alt === null || alt <= 0 || alt > 10000) return commandError('Altitude must be greater than 0 and at most 10,000 m.');
        Object.assign(payload, { lat, lon, alt });
      } else if (cmd === 'takeoff') {
        const alt = readFiniteInput('cmd-alt');
        if (alt === null || alt <= 0 || alt > 10000) return commandError('Altitude must be greater than 0 and at most 10,000 m.');
        payload.alt = alt;
      }
      if (wsSend(payload)) {
        log(`Sent: ${cmd}${cmd === 'fly_to' ? ` to ${payload.lat.toFixed(6)}, ${payload.lon.toFixed(6)} at ${payload.alt}m` : ''}`, 'sys');
      }
    }

    function commandError(message) {
      log(`Command not sent: ${message}`, 'error');
      const feedback = document.getElementById('command-feedback');
      if (feedback) feedback.textContent = `Command not sent: ${message}`;
      return false;
    }

    // ======================== 3D Scene ========================
    const drones = {};
    let selectedDroneId = null;
    let isDropdownOpen = true;
    document.getElementById('drone-list').classList.add('open');
    document.querySelector('button[aria-controls="drone-list"]').setAttribute('aria-expanded', 'true');

    function toggleDropdown() {
      isDropdownOpen = !isDropdownOpen;
      document.getElementById('drone-list').classList.toggle('open', isDropdownOpen);
      document.getElementById('arrow').textContent = isDropdownOpen ? '▲' : '▼';
      document.querySelector('button[aria-controls="drone-list"]').setAttribute('aria-expanded', String(isDropdownOpen));
      if (isDropdownOpen) renderDroneList();
    }

    window.selectDrone = function (id) {
      selectedDroneId = id;
      document.getElementById('tracker-label').textContent = `TRACKING: #${id}`;
      renderDroneList();
    };

    function init3D() {
      const container = document.getElementById('canvas-container');

      scene = new THREE.Scene();
      scene.background = new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue('--canvas-bg').trim());
      scene.fog = new THREE.Fog(getComputedStyle(document.documentElement).getPropertyValue('--canvas-fog').trim(), 100, 5000);

      camera = new THREE.PerspectiveCamera(60, 1, 0.1, 50000);
      camera.position.set(0, 85, 105);

      renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      container.appendChild(renderer.domElement);

      // Tracker
      const trackerEl = document.getElementById('tracker-view');
      trackerRenderer = new THREE.WebGLRenderer({ antialias: true });
      trackerEl.appendChild(trackerRenderer.domElement);
      trackerCamera = new THREE.PerspectiveCamera(50, 1, 0.1, 2000);

      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.05;
      controls.maxDistance = 5000;

      scene.add(new THREE.HemisphereLight(0xd8f2f2, 0x26333b, 1.15));
      const dir = new THREE.DirectionalLight(0xffffff, 1);
      dir.position.set(50, 100, 50);
      scene.add(dir);

      grid = new THREE.GridHelper(1000, 100,
        Number(getComputedStyle(document.documentElement).getPropertyValue('--canvas-grid').trim().replace('#', '0x')),
        Number(getComputedStyle(document.documentElement).getPropertyValue('--canvas-grid-secondary').trim().replace('#', '0x')));
      scene.add(grid);

      ground = new THREE.Mesh(
        new THREE.PlaneGeometry(200000, 200000),
        new THREE.MeshBasicMaterial({ color: getComputedStyle(document.documentElement).getPropertyValue('--canvas-bg').trim(), side: THREE.DoubleSide })
      );
      ground.rotation.x = -Math.PI / 2;
      ground.position.y = -0.5;
      scene.add(ground);

      function animate() {
        requestAnimationFrame(animate);
        controls.update();

        // ---- Flush pending telemetry ----
        if (pendingTelemetry) {
          updateSwarm(pendingTelemetry);
          pendingTelemetry = null;
        }

        // Prop spin
        Object.values(drones).forEach(d => {
          if (d.telemetry && d.telemetry.armed && d.rotors) {
            d.rotors.forEach((rotor, index) => { rotor.rotation.y += index % 2 ? -0.4 : 0.4; });
          }
        });

        // ---- Collision proximity check (runs every frame, cheap pairwise) ----
        checkCollisionProximity();

        if (renderer.domElement.width > 0 && renderer.domElement.height > 0) renderer.render(scene, camera);

        // Tracker
        if (selectedDroneId && drones[selectedDroneId]) {
          const pos = drones[selectedDroneId].mesh.position.clone();
          trackerCamera.position.copy(pos).add(new THREE.Vector3(8, 8, 8));
          trackerCamera.lookAt(pos);
          // Only render tracker if its panel is expanded
          if (!document.getElementById('tracker-body').classList.contains('collapsed') &&
              trackerRenderer.domElement.width > 0 && trackerRenderer.domElement.height > 0) {
            trackerRenderer.render(scene, trackerCamera);
          }
        }
      }
      animate();

      function resizeRendererToBounds(target, targetRenderer, targetCamera) {
        const bounds = target.getBoundingClientRect();
        const width = Math.floor(bounds.width);
        const height = Math.floor(bounds.height);
        if (width <= 0 || height <= 0) return;
        targetRenderer.setSize(width, height, false);
        targetCamera.aspect = width / height;
        targetCamera.updateProjectionMatrix();
      }
      function resizeScenes() {
        resizeRendererToBounds(container, renderer, camera);
        resizeRendererToBounds(trackerEl, trackerRenderer, trackerCamera);
      }
      if (window.ResizeObserver) {
        const sceneResizeObserver = new ResizeObserver(resizeScenes);
        sceneResizeObserver.observe(container);
        sceneResizeObserver.observe(trackerEl);
      }
      window.addEventListener('resize', resizeScenes);
      requestAnimationFrame(resizeScenes);
    }

    // ======================== Collision Proximity Check ========================
    const collisionBanner = document.getElementById('collision-banner');
    let collisionLogThrottle = 0;

    function checkCollisionProximity() {
      const ids = Object.keys(drones);
      if (ids.length < 2) {
        collisionBanner.style.display = 'none';
        return;
      }

      // First pass: determine which drones are in violation across ALL pairs
      const violators = new Set();

      for (let i = 0; i < ids.length; i++) {
        for (let j = i + 1; j < ids.length; j++) {
          const pi = drones[ids[i]].mesh.position;
          const pj = drones[ids[j]].mesh.position;
          const ti = drones[ids[i]].telemetry;
          const tj = drones[ids[j]].telemetry;

          // Don't trigger false collision alarms while drones are still sitting on the ground (< 1m alt)
          const isAirborne = (ti && ti.alt > 1.0) || (tj && tj.alt > 1.0);
          if (!isAirborne) continue;

          // distanceTo gives true 3D Euclidean distance (Y = altitude in scene metres)
          const dist = pi.distanceTo(pj);

          if (dist < COLLISION_WARN_M) {
            violators.add(ids[i]);
            violators.add(ids[j]);

            // Throttled log (at most once every 5 s)
            const now = Date.now();
            if (now - collisionLogThrottle > 5000) {
              collisionLogThrottle = now;
              log(`⚠ COLLISION: Drone #${ids[i]} & #${ids[j]} are ${dist.toFixed(1)}m apart!`, 'error');
            }
          }
        }
      }

      // Second pass: apply tints based on accumulated violation set
      ids.forEach(id => {
        setDroneTint(id, violators.has(id) ? DRONE_COLLISION_COLOR : DRONE_NORMAL_COLOR);
      });

      collisionBanner.style.display = violators.size > 0 ? 'block' : 'none';
    }

    init3D();

    const defaultCameraPosition = new THREE.Vector3(0, 85, 105);
    window.fitFleet = function () {
      const entries = Object.values(drones).filter(drone => drone.telemetry);
      const points = entries.map(drone => drone.mesh.position);
      const center = points.length
        ? points.reduce((sum, point) => sum.add(point), new THREE.Vector3()).multiplyScalar(1 / points.length)
        : new THREE.Vector3(0, 0, 0);
      const halfWidth = points.length ? Math.max(...points.map(point => Math.abs(point.x - center.x))) : 20;
      const halfDepth = points.length ? Math.max(...points.map(point => Math.abs(point.z - center.z))) : 20;
      const halfHeight = points.length ? Math.max(...points.map(point => Math.abs(point.y - center.y))) : 5;
      const bounds = document.getElementById('canvas-container').getBoundingClientRect();
      const aspect = bounds.width > 0 && bounds.height > 0 ? bounds.width / bounds.height : 1;
      const fov = THREE.MathUtils.degToRad(camera.fov);
      const horizontalFov = 2 * Math.atan(Math.tan(fov / 2) * aspect);
      const distance = Math.max(20, (Math.max(halfWidth / Math.tan(horizontalFov / 2), halfDepth / Math.tan(fov / 2), halfHeight) + 8) * 1.35);
      const direction = new THREE.Vector3(1, 0.85, 1).normalize();
      controls.target.copy(center);
      camera.up.set(0, 1, 0);
      camera.position.copy(center).addScaledVector(direction, distance);
      camera.near = Math.max(0.1, distance / 1000);
      camera.far = Math.max(50000, distance * 20);
      camera.updateProjectionMatrix();
      controls.update();
    };
    window.setTopView = function () {
      const center = controls.target.clone();
      const distance = Math.max(20, camera.position.distanceTo(center));
      camera.position.set(center.x, center.y + distance, center.z + 0.001);
      camera.up.set(0, 0, -1);
      camera.lookAt(center);
      controls.update();
    };
    window.resetView = function () {
      controls.target.set(0, 0, 0);
      camera.up.set(0, 1, 0);
      camera.position.copy(defaultCameraPosition);
      camera.near = 0.1;
      camera.far = 50000;
      camera.updateProjectionMatrix();
      controls.update();
    };

    function setDroneTint(sysid, color) {
      if (!drones[sysid]) return;
      const mat = drones[sysid].bodyMaterial;
      if (mat && mat.color.getHex() !== color) {
        mat.color.setHex(color);
        mat.emissive.setHex(
          color === DRONE_COLLISION_COLOR ? DRONE_COLLISION_EMISSIVE : DRONE_NORMAL_EMISSIVE
        );
      }
    }

    // ======================== Drone 3D Objects ========================
    function createDrone(sysid) {
      const group = new THREE.Group();
      const bodyMaterial = new THREE.MeshPhongMaterial({
        color: DRONE_NORMAL_COLOR, emissive: DRONE_NORMAL_EMISSIVE, emissiveIntensity: 0.5
      });
      const body = new THREE.Group();
      const bodyTube = new THREE.Mesh(new THREE.CylinderGeometry(0.22, 0.22, 0.5, 16), bodyMaterial);
      bodyTube.rotation.x = Math.PI / 2;
      body.add(bodyTube);
      const bodyNose = new THREE.Mesh(new THREE.SphereGeometry(0.22, 16, 10), bodyMaterial);
      bodyNose.position.z = -0.25;
      body.add(bodyNose);
      const bodyTail = new THREE.Mesh(new THREE.SphereGeometry(0.22, 16, 10), bodyMaterial);
      bodyTail.position.z = 0.25;
      body.add(bodyTail);
      body.position.y = 0.02;
      group.add(body);

      const armMaterial = new THREE.MeshPhongMaterial({ color: 0x9aadaa, shininess: 55 });
      const motorMaterial = new THREE.MeshPhongMaterial({ color: 0xa9bdc5, shininess: 70 });
      const rotorBladeMaterial = new THREE.MeshBasicMaterial({ color: 0xc3d1cf, transparent: true, opacity: 0.78 });
      const motorPoints = [
        { x: 0.72, z: 0.72 }, { x: -0.72, z: 0.72 },
        { x: 0.72, z: -0.72 }, { x: -0.72, z: -0.72 },
      ];
      const hubMaterial = new THREE.MeshPhongMaterial({ color: 0x354a53, shininess: 80 });
      const canopyMaterial = new THREE.MeshPhongMaterial({ color: 0xe5a33a, shininess: 65 });
      const armGeometry = new THREE.CylinderGeometry(0.035, 0.045, 1, 8);
      const rotors = [];
      motorPoints.forEach(point => {
        const end = new THREE.Vector3(point.x, 0, point.z);
        const arm = new THREE.Mesh(armGeometry, armMaterial);
        arm.position.copy(end).multiplyScalar(0.5);
        arm.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), end.clone().normalize());
        arm.scale.y = end.length();
        group.add(arm);

        const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.12, 0.11, 16), hubMaterial);
        hub.position.set(point.x, 0.06, point.z);
        group.add(hub);

        const rotor = new THREE.Group();
        rotor.position.set(point.x, 0.11, point.z);
        const motor = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.12, 16), motorMaterial);
        rotor.add(motor);
        for (let bladeIndex = 0; bladeIndex < 2; bladeIndex++) {
          const blade = new THREE.Mesh(new THREE.BoxGeometry(0.48, 0.018, 0.055), rotorBladeMaterial);
          blade.position.y = 0.075;
          blade.rotation.y = bladeIndex * Math.PI / 2;
          rotor.add(blade);
        }
        group.add(rotor);
        rotors.push(rotor);
      });

      const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.28, 8), armMaterial);
      mast.position.set(0, 0.2, -0.12);
      group.add(mast);
      const gps = new THREE.Mesh(new THREE.SphereGeometry(0.11, 12, 8), canopyMaterial);
      gps.position.set(0, 0.37, -0.12);
      group.add(gps);
      const nose = new THREE.Mesh(new THREE.ConeGeometry(0.16, 0.26, 8), canopyMaterial);
      nose.rotation.x = Math.PI / 2;
      nose.position.set(0, 0.02, -0.38);
      group.add(nose);

      // Label
      const lCanvas = document.createElement('canvas');
      lCanvas.width = 256; lCanvas.height = 96;
      const ctx = lCanvas.getContext('2d');
      ctx.fillStyle = 'rgba(8, 18, 23, 0.86)';
      ctx.beginPath();
      ctx.roundRect(3, 3, 250, 90, 20);
      ctx.fill();
      ctx.strokeStyle = '#55d6be';
      ctx.lineWidth = 5;
      ctx.stroke();
      ctx.fillStyle = '#d8fff8';
      ctx.font = 'bold 58px monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(`#${sysid}`, 128, 49);
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(lCanvas) })
      );
      sprite.position.y = 1.65;
      // Offset in screen space so overhead labels never cover the airframe.
      sprite.center.set(0.5, -0.65);
      sprite.scale.set(2.8, 1.05, 1);
      group.add(sprite);

      group.position.set(0, 0, 0);
      scene.add(group);
      return { mesh: group, bodyMaterial, rotors };
    }

    // ======================== Telemetry Processing ========================
    window.homeRef = null;
    let lastListUpdate = 0;
    let listRenderTimer = null;

    function updateSwarm(data) {
      // Set home reference from first valid fix
      if (!window.homeRef) {
        const firstId = Object.keys(data)[0];
        if (firstId && Math.abs(data[firstId].lat) > 0.1) {
          window.homeRef = { lat: data[firstId].lat, lon: data[firstId].lon };
          log(`Home ref: ${window.homeRef.lat.toFixed(6)}, ${window.homeRef.lon.toFixed(6)}`, 'sys');
        }
      }

      let newDroneAdded = false;

      for (const [sysid, t] of Object.entries(data)) {
        if (!drones[sysid]) {
          drones[sysid] = createDrone(sysid);
          log(`New drone detected: #${sysid}`, 'sys');
          if (!selectedDroneId) selectDrone(sysid);
          newDroneAdded = true;
        }

        drones[sysid].telemetry = t;

        if (window.homeRef) {
          const latDiff = t.lat - window.homeRef.lat;
          const lonDiff = t.lon - window.homeRef.lon;
          const cosLat = Math.cos(window.homeRef.lat * Math.PI / 180);

          // Position in 3D scene (X = East, Z = South, Y = Altitude AGL)
          const x = lonDiff * 111320 * cosLat;
          const z = -latDiff * 110574;
          const y = (t.alt !== undefined) ? t.alt : 0;

          drones[sysid].mesh.position.set(x, Math.max(0, y), z);
          drones[sysid].mesh.rotation.x = t.pitch * (Math.PI / 180);
          drones[sysid].mesh.rotation.z = -t.roll * (Math.PI / 180);
          drones[sysid].mesh.rotation.y = -t.yaw * (Math.PI / 180);
        }
      }

      // Update stats
      const allDrones = Object.values(drones);
      const activeCount = allDrones.length;
      document.getElementById('active-count').textContent = activeCount;
      document.getElementById('sb-drones-val').textContent = activeCount;
      document.getElementById('drone-list-count').textContent = `${activeCount} in fleet`;

      let totalAlt = 0;
      allDrones.forEach(d => {
        const altitude = d.telemetry && Number(d.telemetry.alt);
        if (Number.isFinite(altitude)) totalAlt += altitude;
      });
      document.getElementById('avg-alt').textContent =
        activeCount > 0 ? (totalAlt / activeCount).toFixed(1) + 'm' : '0.0m';

      // Leader position display
      const leaderSysid = currentLeaderSysid === -1
        ? (Object.keys(drones).sort((a, b) => parseInt(a) - parseInt(b))[0] || null)
        : (currentLeaderSysid === null ? null : String(currentLeaderSysid));

      const leaderPosEl = document.getElementById('leader-pos');
      if (leaderSysid && drones[leaderSysid] && drones[leaderSysid].telemetry) {
        const lt = drones[leaderSysid].telemetry;
        leaderPosEl.textContent =
          `L:${lt.lat.toFixed(5)} LN:${lt.lon.toFixed(5)} A:${lt.alt.toFixed(1)}m`;
      } else {
        leaderPosEl.textContent = 'WAITING…';
      }

      if (newDroneAdded) {
        if (listRenderTimer) clearTimeout(listRenderTimer);
        listRenderTimer = null;
        updateLeaderSelect();
        renderDroneList();
        lastListUpdate = Date.now();
      }
      if (newDroneAdded) window.fitFleet();

      if (isDropdownOpen && !newDroneAdded) {
        const now = Date.now();
        if (now - lastListUpdate > DRONE_LIST_UPDATE_MS) {
          renderDroneList();
          lastListUpdate = now;
          if (listRenderTimer) clearTimeout(listRenderTimer);
          listRenderTimer = null;
        } else if (!listRenderTimer) {
          listRenderTimer = setTimeout(() => {
            listRenderTimer = null;
            if (isDropdownOpen) {
              renderDroneList();
              lastListUpdate = Date.now();
            }
          }, DRONE_LIST_UPDATE_MS - (now - lastListUpdate));
        }
      }
    }

    function renderDroneList() {
      const sortedIds = Object.keys(drones).sort((a, b) => parseInt(a) - parseInt(b));
      const leaderSysid = currentLeaderSysid === -1
        ? (sortedIds[0] || null)
        : (currentLeaderSysid === null ? null : String(currentLeaderSysid));
      const list = document.getElementById('drone-list');
      const focused = document.activeElement;
      let focusedDroneId = focused && focused.dataset ? focused.dataset.droneId : null;
      const currentRows = new Map([...list.querySelectorAll('button.drone-row')].map(row => [row.dataset.droneId, row]));
      const requiredIds = new Set(sortedIds);
      currentRows.forEach((row, id) => {
        if (!requiredIds.has(id)) {
          row.remove();
          currentRows.delete(id);
        }
      });
      sortedIds.forEach(id => {
        const t = drones[id].telemetry;
        if (!t) return;
        let row = currentRows.get(id);
        if (!row) {
          row = document.createElement('button');
          row.type = 'button';
          row.className = 'drone-row';
          row.dataset.droneId = id;
          const header = document.createElement('span');
          header.className = 'header';
          const title = document.createElement('span');
          title.className = 'drone-name';
          const badges = document.createElement('span');
          badges.className = 'drone-badges';
          const leaderBadge = document.createElement('span');
          leaderBadge.className = 'drone-badge leader-badge';
          leaderBadge.textContent = 'LEADER';
          const stateDot = document.createElement('span');
          stateDot.className = 'drone-state-dot';
          stateDot.setAttribute('aria-hidden', 'true');
          const stateText = document.createElement('span');
          stateText.className = 'drone-state';
          badges.append(leaderBadge, stateDot, stateText);
          header.append(title, badges);

          const details = document.createElement('span');
          details.className = 'details';
          const values = [
            ['ALT', 'altitude'], ['H SPEED', 'horizontal-speed'], ['V SPEED', 'vertical-speed'],
          ].map(([label, key]) => {
            const metric = document.createElement('span');
            metric.className = 'drone-metric';
            const caption = document.createElement('span');
            caption.className = 'drone-metric-label';
            caption.textContent = label;
            const value = document.createElement('span');
            value.className = 'drone-metric-value';
            value.dataset.metric = key;
            metric.append(caption, value);
            details.appendChild(metric);
            return value;
          });
          const footer = document.createElement('span');
          footer.className = 'drone-footer';
          const position = document.createElement('span');
          position.className = 'drone-coordinate';
          const msl = document.createElement('span');
          msl.className = 'drone-msl';
          footer.append(position, msl);
          row.append(header, details, footer);
          row._title = title;
          row._leaderBadge = leaderBadge;
          row._stateDot = stateDot;
          row._stateText = stateText;
          row._values = values;
          row._position = position;
          row._msl = msl;
          row.addEventListener('click', event => {
            event.stopPropagation();
            window.selectDrone(row.dataset.droneId);
          });
          currentRows.set(id, row);
        }
        const selected = id === selectedDroneId;
        const isLeader = id === leaderSysid;
        row.classList.toggle('selected', selected);
        row.classList.toggle('leader-row', isLeader);
        row.setAttribute('aria-pressed', String(selected));
        row._title.textContent = `Drone #${id}`;
        row._leaderBadge.hidden = !isLeader;
        const armed = Boolean(t.armed);
        row.classList.toggle('armed', armed);
        row._stateDot.className = `drone-state-dot${armed ? ' armed' : ''}`;
        row._stateText.textContent = armed ? 'ARMED' : 'DISARMED';
        const finite = value => Number.isFinite(Number(value)) ? Number(value) : null;
        const lat = finite(t.lat), lon = finite(t.lon), alt = finite(t.alt), msl = finite(t.alt_msl);
        const hSpeed = finite(t.h_speed), vSpeed = finite(t.v_speed);
        const formatted = [
          alt === null ? '—' : `${alt.toFixed(1)} m`,
          hSpeed === null ? '—' : `${hSpeed.toFixed(1)} m/s`,
          vSpeed === null ? '—' : `${vSpeed.toFixed(1)} m/s`,
        ];
        row._values.forEach((node, index) => { node.textContent = formatted[index]; });
        row._position.textContent = `GPS  ${lat === null ? '—' : lat.toFixed(5)}, ${lon === null ? '—' : lon.toFixed(5)}`;
        row._msl.textContent = `MSL  ${msl === null ? '—' : `${msl.toFixed(1)} m`}`;
        row.setAttribute('aria-label', `Drone ${id}${isLeader ? ', leader' : ''}, ${armed ? 'armed' : 'disarmed'}. Altitude ${formatted[0]}, horizontal speed ${formatted[1]}, vertical speed ${formatted[2]}. Latitude ${lat ?? 'unavailable'}, longitude ${lon ?? 'unavailable'}, mean sea level altitude ${msl === null ? 'unavailable' : `${msl.toFixed(1)} meters`}.`);
        const index = sortedIds.indexOf(id);
        if (list.children[index] !== row) list.insertBefore(row, list.children[index] || null);
      });
      if (focusedDroneId && currentRows.has(focusedDroneId) && document.activeElement !== currentRows.get(focusedDroneId)) {
        currentRows.get(focusedDroneId).focus({ preventScroll: true });
      }
    }

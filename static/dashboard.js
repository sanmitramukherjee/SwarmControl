// ======================== Configuration Constants ========================
    const WS_URL                   = 'ws://localhost:8001';
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

    // ======================== Panel Toggle ========================
    function togglePanel(bodyId, arrowId) {
      const body = document.getElementById(bodyId);
      const arrow = document.getElementById(arrowId);
      const collapsed = body.classList.toggle('collapsed');
      arrow.textContent = collapsed ? '▼' : '▲';
    }

    // ======================== Sidebar Toggle ========================
    function toggleSidebar(evt) {
      if (evt) evt.stopPropagation();
      const sidebar = document.getElementById('sidebar');
      sidebar.classList.toggle('collapsed');
    }

    // Bind sidebar toggle to any click inside the sidebar
    const _sidebarEl = document.getElementById('sidebar');
    if (_sidebarEl) {
      _sidebarEl.addEventListener('click', toggleSidebar);
    }

    // ======================== WebSocket with Exponential Backoff ========================
    let ws = null;
    let wsRetryDelay = WS_INITIAL_RETRY_DELAY;
    let wsConnected = false;

    function connectWS() {
      ws = new WebSocket(WS_URL);

      ws.onopen = () => {
        wsConnected = true;
        wsRetryDelay = WS_INITIAL_RETRY_DELAY;
        document.getElementById('sb-ws-val').textContent = 'CONNECTED';
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
        wsConnected = false;
        document.getElementById('sb-ws-val').textContent = 'DISCONNECTED';
        document.getElementById('reconnect-banner').style.display = 'block';
        log(`WebSocket closed. Retrying in ${(wsRetryDelay/1000).toFixed(1)}s…`, 'error');
        setTimeout(() => {
          wsRetryDelay = Math.min(wsRetryDelay * 2, WS_MAX_DELAY);
          connectWS();
        }, wsRetryDelay);
      };

      ws.onerror = () => {
        document.getElementById('sb-ws-val').textContent = 'ERROR';
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
    let currentLeaderSysid = 1;  // Hardcoded default leader is Drone 1

    function applyStatus(data) {
      if (data.mode !== undefined) {
        document.getElementById('sb-mode-val').textContent = data.mode;
      }
      if (data.leader !== undefined) {
        currentLeaderSysid = parseInt(data.leader);
        const label = currentLeaderSysid === -1 ? 'AUTO' : `#${currentLeaderSysid}`;
        document.getElementById('sb-leader-val').textContent = label;
        syncLeaderSelect();
      }
      if (data.drone_count !== undefined) {
        document.getElementById('sb-drones-val').textContent = data.drone_count;
      }
      if (data.manual_mode !== undefined) {
        isManualModeActive = Boolean(data.manual_mode);
        updateManualModeUI();
      }
    }

    let isManualModeActive = false;

    function toggleManualMode() {
      const fState = getFormationState();
      const payload = {
        type: 'swarm_command',
        cmd: 'manual_mode',
        ...fState,
        alt: parseFloat(document.getElementById('cmd-alt').value) || 10,
      };
      wsSend(payload);
    }

    function updateManualModeUI() {
      const btn = document.getElementById('btn-manual-mode');
      if (!btn) return;
      if (isManualModeActive) {
        btn.classList.add('active');
        btn.textContent = 'MANUAL MODE: ACTIVE (CLICK TO STOP)';
      } else {
        btn.classList.remove('active');
        btn.textContent = 'MANUAL MODE (RC FOLLOW)';
      }
    }

    // ======================== Leader Control ========================
    function onLeaderChange(val) {
      currentLeaderSysid = parseInt(val);
      const label = currentLeaderSysid === -1 ? 'AUTO' : `#${currentLeaderSysid}`;
      document.getElementById('sb-leader-val').textContent = label;
      wsSend({ type: 'set_leader', sysid: currentLeaderSysid });
      log(`Leader set to ${label}`, 'sys');
    }

    function syncLeaderSelect() {
      const sel = document.getElementById('leader-select');
      sel.value = String(currentLeaderSysid);
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
        ws.send(JSON.stringify(payload));
      } else {
        log('WebSocket not connected – command dropped.', 'error');
      }
    }

    function swarmCmd(cmd) {
      const fState = getFormationState();
      const payload = {
        type: 'swarm_command',
        cmd,
        ...fState,
        lat: parseFloat(document.getElementById('cmd-lat').value) || 0,
        lon: parseFloat(document.getElementById('cmd-lon').value) || 0,
        alt: parseFloat(document.getElementById('cmd-alt').value) || 5,
      };
      wsSend(payload);
      log(`Sent: ${cmd} | formation=${payload.formation} pattern=${payload.pattern} spacing=${payload.spacing_m}m`, 'sys');
    }

    // ======================== 3D Scene ========================
    const drones = {};
    let scene, camera, renderer, controls;
    let trackerRenderer, trackerCamera;
    let selectedDroneId = null;
    let isDropdownOpen = false;

    function toggleDropdown() {
      isDropdownOpen = !isDropdownOpen;
      document.getElementById('drone-list').classList.toggle('open', isDropdownOpen);
      document.getElementById('arrow').textContent = isDropdownOpen ? '▲' : '▼';
      document.querySelector('.overlay h2 button').setAttribute('aria-expanded', String(isDropdownOpen));
    }

    window.selectDrone = function (id) {
      selectedDroneId = id;
      document.getElementById('tracker-label').textContent = `TRACKING: #${id}`;
      renderDroneList();
      if (drones[id] && drones[id].telemetry) {
        document.getElementById('cmd-lat').value = drones[id].telemetry.lat.toFixed(6);
        document.getElementById('cmd-lon').value = drones[id].telemetry.lon.toFixed(6);
      }
    };

    function init3D() {
      const container = document.getElementById('canvas-container');

      scene = new THREE.Scene();
      scene.background = new THREE.Color(0x101820);
      scene.fog = new THREE.Fog(0x101820, 100, 5000);

      camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 50000);
      camera.position.set(0, 85, 105);

      renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.setSize(container.clientWidth, container.clientHeight);
      renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      container.appendChild(renderer.domElement);

      // Tracker
      const trackerEl = document.getElementById('tracker-view');
      trackerRenderer = new THREE.WebGLRenderer({ antialias: true });
      trackerRenderer.setSize(trackerEl.clientWidth, trackerEl.clientHeight);
      trackerEl.appendChild(trackerRenderer.domElement);
      trackerCamera = new THREE.PerspectiveCamera(50, trackerEl.clientWidth / trackerEl.clientHeight, 0.1, 2000);

      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.dampingFactor = 0.05;
      controls.maxDistance = 5000;

      scene.add(new THREE.HemisphereLight(0xd8f2f2, 0x26333b, 1.15));
      const dir = new THREE.DirectionalLight(0xffffff, 1);
      dir.position.set(50, 100, 50);
      scene.add(dir);

      scene.add(new THREE.GridHelper(1000, 100, 0x39545d, 0x23333b));

      const ground = new THREE.Mesh(
        new THREE.PlaneGeometry(200000, 200000),
        new THREE.MeshBasicMaterial({ color: 0x152129, side: THREE.DoubleSide })
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
        Object.values(drones).forEach(d => { if (d.props) d.props.rotation.y += 0.4; });

        // ---- Collision proximity check (runs every frame, cheap pairwise) ----
        checkCollisionProximity();

        renderer.render(scene, camera);

        // Tracker
        if (selectedDroneId && drones[selectedDroneId]) {
          const pos = drones[selectedDroneId].mesh.position.clone();
          trackerCamera.position.copy(pos).add(new THREE.Vector3(8, 8, 8));
          trackerCamera.lookAt(pos);
          // Only render tracker if its panel is expanded
          if (!document.getElementById('tracker-body').classList.contains('collapsed')) {
            trackerRenderer.render(scene, trackerCamera);
          }
        }
      }
      animate();

      window.addEventListener('resize', () => {
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
      });
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

    // Cache body materials per drone for fast tinting
    const _droneTintCache = {};
    function setDroneTint(sysid, color) {
      if (!drones[sysid]) return;
      if (!_droneTintCache[sysid]) {
        // Find the body mesh (first child that is a Mesh with MeshPhongMaterial)
        drones[sysid].mesh.children.forEach(child => {
          if (child.isMesh && child.material && child.material.isMeshPhongMaterial) {
            _droneTintCache[sysid] = child.material;
          }
        });
      }
      const mat = _droneTintCache[sysid];
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

      const mat = new THREE.MeshPhongMaterial({
        color: DRONE_NORMAL_COLOR, emissive: DRONE_NORMAL_EMISSIVE, emissiveIntensity: 0.5
      });
      const body = new THREE.Mesh(new THREE.SphereGeometry(0.2, 16, 10), mat);
      body.scale.set(1.25, 0.55, 1);
      group.add(body);

      const armMat = new THREE.MeshPhongMaterial({ color: 0xa8b8b7, shininess: 45 });
      const armGeom = new THREE.CylinderGeometry(0.035, 0.055, 1.1, 8);
      const arm1 = new THREE.Mesh(armGeom, armMat); arm1.rotation.z = Math.PI / 2; arm1.rotation.y = Math.PI / 4;
      const arm2 = new THREE.Mesh(armGeom, armMat); arm2.rotation.z = Math.PI / 2; arm2.rotation.y = -Math.PI / 4;
      group.add(arm1); group.add(arm2);

      const propGroup = new THREE.Group();
      const propGeom = new THREE.CylinderGeometry(0.22, 0.22, 0.018, 16);
      const propMat = new THREE.MeshBasicMaterial({ color: 0xc3d1cf, transparent: true, opacity: 0.68 });
      [{ x: 0.42, z: 0.42 }, { x: -0.42, z: -0.42 }, { x: 0.42, z: -0.42 }, { x: -0.42, z: 0.42 }].forEach(p => {
        const prop = new THREE.Mesh(propGeom, propMat);
        prop.position.set(p.x, 0.1, p.z);
        propGroup.add(prop);
      });
      group.add(propGroup);

      // Label
      const lCanvas = document.createElement('canvas');
      lCanvas.width = 64; lCanvas.height = 32;
      const ctx = lCanvas.getContext('2d');
      ctx.fillStyle = '#55d6be';
      ctx.font = 'bold 20px monospace';
      ctx.fillText(`#${sysid}`, 4, 24);
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(lCanvas) })
      );
      sprite.position.y = 1.2;
      sprite.scale.set(2, 1, 1);
      group.add(sprite);

      group.position.set(0, 0, 0);
      scene.add(group);
      return { mesh: group, props: propGroup };
    }

    // ======================== Telemetry Processing ========================
    window.homeRef = null;
    let lastListUpdate = 0;

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

      let totalAlt = 0;
      allDrones.forEach(d => { if (d.telemetry) totalAlt += d.telemetry.alt; });
      document.getElementById('avg-alt').textContent =
        activeCount > 0 ? (totalAlt / activeCount).toFixed(1) + 'm' : '0.0m';

      // Leader position display
      const leaderSysid = currentLeaderSysid === -1
        ? (Object.keys(drones).sort((a, b) => parseInt(a) - parseInt(b))[0] || null)
        : String(currentLeaderSysid);

      const leaderPosEl = document.getElementById('leader-pos');
      if (leaderSysid && drones[leaderSysid] && drones[leaderSysid].telemetry) {
        const lt = drones[leaderSysid].telemetry;
        leaderPosEl.textContent =
          `L:${lt.lat.toFixed(5)} LN:${lt.lon.toFixed(5)} A:${lt.alt.toFixed(1)}m`;
      } else {
        leaderPosEl.textContent = 'WAITING…';
      }

      if (newDroneAdded) updateLeaderSelect();

      if (isDropdownOpen) {
        const now = Date.now();
        if (now - lastListUpdate > DRONE_LIST_UPDATE_MS) {
          renderDroneList();
          lastListUpdate = now;
        }
      }
    }

    function renderDroneList() {
      const sortedIds = Object.keys(drones).sort((a, b) => parseInt(a) - parseInt(b));
      const leaderSysid = currentLeaderSysid === -1
        ? (sortedIds[0] || null)
        : String(currentLeaderSysid);

      let html = '';
      sortedIds.forEach(id => {
        const t = drones[id].telemetry;
        const selected = id === selectedDroneId ? 'selected' : '';
        const isLeader = id === leaderSysid ? 'leader-row' : '';
        const leaderBadge = id === leaderSysid ? ' ★LEADER' : '';
        if (t) {
          html += `
            <div class="drone-row ${selected} ${isLeader}"
                 onclick="event.stopPropagation();selectDrone('${id}')">
              <div class="header">DRONE #${id}${leaderBadge}</div>
              <div class="details">
                <span>Lat: ${t.lat.toFixed(5)}</span>
                <span>Lon: ${t.lon.toFixed(5)}</span>
                <span>AGL: ${t.alt.toFixed(1)}m</span>
                <span>MSL: ${(t.alt_msl||0).toFixed(1)}m</span>
                <span>HSpd: ${t.h_speed.toFixed(1)}</span>
                <span>VSpd: ${t.v_speed.toFixed(1)}</span>
              </div>
            </div>`;
        }
      });
      document.getElementById('drone-list').innerHTML = html;
    }
  
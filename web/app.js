/**
 * The recorder's browser front end. No build step: plain modules, and three.js
 * comes straight off esm.sh so the Pi never has to run a bundler.
 */

const element = (id) => document.getElementById(id)

const state = {
    /** @type {object|null} last full status payload from /api/status */
    settings: null,
    recording: { active: false },
    previewTopics: [],
    /** the object URL currently shown, revoked when the next frame lands */
    previewUrl: null,
    urdfXml: null,
    /** set while a settings PUT is in flight, so the poll does not clobber typing */
    savingSettings: false,
}

// -- small helpers --------------------------------------------------------

const bytesToText = (bytes) => {
    if (bytes < 1024) {
        return `${bytes} B`
    }
    const units = ["KB", "MB", "GB", "TB"]
    let value = bytes / 1024
    let index = 0
    while (value >= 1024 && index < units.length - 1) {
        value = value / 1024
        index = index + 1
    }
    return `${value.toFixed(1)} ${units[index]}`
}

const toast = (message, bad) => {
    const box = element("toast")
    box.textContent = message
    box.hidden = false
    box.classList.toggle("toast-bad", Boolean(bad))
    clearTimeout(toast.timer)
    toast.timer = setTimeout(() => { box.hidden = true }, 4000)
}

/**
 * Every call goes through here so a backend error message reaches the operator
 * instead of vanishing into the console.
 */
const request = async (path, options) => {
    const response = await fetch(path, options)
    const text = await response.text()
    let body = null
    if (text) {
        try {
            body = JSON.parse(text)
        } catch (error) {
            body = { error: text }
        }
    }
    if (!response.ok) {
        throw new Error((body && body.error) || `${response.status} ${response.statusText}`)
    }
    return body
}

const postJson = (path, body) => request(path, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body === undefined ? {} : body),
})

/** Reads a dotted path like `livox.naming.topic_prefix` out of an object. */
const readPath = (root, path) => path.split(".").reduce(
    (value, key) => (value === null || value === undefined ? undefined : value[key]),
    root,
)

/** Writes a dotted path, creating nothing — every path already exists in Settings. */
const writePath = (root, path, value) => {
    const keys = path.split(".")
    const last = keys.pop()
    const target = keys.reduce((value, key) => value[key], root)
    target[last] = value
}

// -- settings binding -----------------------------------------------------

/**
 * Pushes the whole Settings struct back. The server is the only validator, so a
 * rejected change is reported rather than silently kept in the page.
 */
const saveSettings = async () => {
    state.savingSettings = true
    try {
        const payload = await request("/api/settings", {
            method: "PUT",
            headers: { "content-type": "application/json" },
            body: JSON.stringify(state.settings),
        })
        applyStatus(payload)
    } catch (error) {
        toast(error.message, true)
        await refreshStatus()
    } finally {
        state.savingSettings = false
    }
}

const readControl = (control) => {
    if (control.type === "checkbox") {
        return control.checked
    }
    if (control.type === "number") {
        return Number(control.value)
    }
    // An empty optional field means "unset", not the empty string; the lidar
    // address is the case that matters, where "" would be a bad IP.
    if (control.dataset.setting === "livox.lidar_address" && control.value.trim() === "") {
        return null
    }
    return control.value
}

const bindSettingControl = (control) => {
    const path = control.dataset.setting
    const commit = () => {
        if (!state.settings) {
            return
        }
        writePath(state.settings, path, readControl(control))
        saveSettings()
    }
    const immediate = control.type === "checkbox" || control.tagName === "SELECT"
    control.addEventListener(immediate ? "change" : "change", commit)
}

const fillSettingControls = () => {
    for (const control of document.querySelectorAll("[data-setting]")) {
        const value = readPath(state.settings, control.dataset.setting)
        if (document.activeElement === control) {
            continue
        }
        if (control.type === "checkbox") {
            control.checked = Boolean(value)
        } else {
            control.value = value === null || value === undefined ? "" : value
        }
    }
}

// -- camera settings panels ----------------------------------------------

const cameraFields = [
    { key: "enabled", label: "Enabled", type: "checkbox" },
    { key: "depth", label: "Depth", type: "checkbox" },
    { key: "color", label: "Colour", type: "checkbox" },
    { key: "infrared", label: "Infrared", type: "checkbox" },
    { key: "imu", label: "IMU", type: "checkbox" },
    { key: "emitter", label: "IR emitter", type: "checkbox" },
    { key: "align_depth_to_color", label: "Align depth to colour", type: "checkbox" },
]

const cameraNumbers = [
    { key: "width", label: "Width" },
    { key: "height", label: "Height" },
    { key: "frame_rate", label: "Frame rate" },
]

/**
 * Both cameras expose the same knobs, so the panels are generated rather than
 * written twice and drifting apart.
 */
const buildCameraSettings = () => {
    const host = element("camera-settings")
    if (host.dataset.built) {
        return
    }
    host.dataset.built = "yes"
    for (const [kind, title] of [["realsense", "RealSense"], ["orbbec", "Orbbec"]]) {
        const group = document.createElement("div")
        group.innerHTML = `<h3>${title}</h3>`

        const toggles = document.createElement("div")
        toggles.className = "row wrap"
        for (const field of cameraFields) {
            const label = document.createElement("label")
            label.className = "check"
            label.innerHTML = `<input type="checkbox" data-setting="${kind}.${field.key}"><span>${field.label}</span>`
            toggles.append(label)
        }
        group.append(toggles)

        const numbers = document.createElement("div")
        numbers.className = "row wrap"
        for (const field of cameraNumbers) {
            const label = document.createElement("label")
            label.innerHTML = `${field.label} <input type="number" min="1" data-setting="${kind}.${field.key}">`
            numbers.append(label)
        }
        group.append(numbers)

        const naming = document.createElement("div")
        naming.className = "row wrap"
        naming.innerHTML = `
            <label>Topic prefix <input type="text" data-setting="${kind}.naming.topic_prefix"></label>
            <label>Frame prefix <input type="text" data-setting="${kind}.naming.frame_prefix"></label>
            <label>Serial <input type="text" data-setting="${kind}.serial" placeholder="any"></label>`
        group.append(naming)

        host.append(group)
    }
    for (const control of host.querySelectorAll("[data-setting]")) {
        bindSettingControl(control)
    }
}

// -- rendering ------------------------------------------------------------

const renderSensors = (sensors) => {
    const host = element("sensor-list")
    host.textContent = ""
    for (const [kind, status] of Object.entries(sensors)) {
        const row = document.createElement("div")
        row.className = "sensor"
        row.dataset.sensor = kind

        const name = document.createElement("strong")
        name.textContent = kind
        const detail = document.createElement("span")
        detail.className = "detail"
        detail.textContent = status.error ? status.error : status.detail
        detail.classList.toggle("bad", Boolean(status.error))

        const pill = document.createElement("span")
        pill.className = `pill ${status.running ? "pill-good" : "pill-idle"}`
        pill.textContent = status.running ? "engaged" : "disengaged"
        pill.dataset.role = "state"

        const button = document.createElement("button")
        button.className = "secondary"
        button.textContent = status.running ? "Disengage" : "Engage"
        button.dataset.role = "toggle"
        button.addEventListener("click", async () => {
            const action = status.running ? "disengage" : "engage"
            button.disabled = true
            try {
                renderSensors(await postJson(`/api/sensors/${kind}/${action}`))
            } catch (error) {
                toast(error.message, true)
                button.disabled = false
            }
        })

        row.append(name, pill, detail, button)
        host.append(row)
    }
}

const renderRecording = (recording) => {
    state.recording = recording
    element("recording-state").textContent = recording.active ? "recording" : "idle"
    element("recording-state").classList.toggle("good", recording.active)
    element("recording-seconds").textContent = `${recording.seconds.toFixed(1)} s`
    element("recording-messages").textContent = recording.messages.toLocaleString()
    element("recording-bytes").textContent = bytesToText(recording.bytes)
    element("recording-dropped").textContent = recording.dropped.toLocaleString()
    element("recording-dropped").classList.toggle("bad", recording.dropped > 0)
    element("recording-path").textContent = recording.path || ""
    const toggle = element("record-toggle")
    toggle.textContent = recording.active ? "Stop recording" : "Start recording"
    toggle.classList.toggle("recording", recording.active)
    document.body.classList.toggle("is-recording", recording.active)
}

const renderStreams = (streams) => {
    const body = element("stream-rows")
    body.textContent = ""
    for (const stream of streams) {
        const row = document.createElement("tr")
        row.dataset.topic = stream.topic
        const dropped = document.createElement("td")
        dropped.textContent = stream.dropped.toLocaleString()
        dropped.classList.toggle("bad", stream.dropped > 0)
        for (const text of [stream.topic, stream.hz.toFixed(1), stream.total.toLocaleString()]) {
            const cell = document.createElement("td")
            cell.textContent = text
            row.append(cell)
        }
        row.append(dropped)
        body.append(row)
    }
    if (streams.length === 0) {
        const row = document.createElement("tr")
        row.innerHTML = `<td colspan="4" class="empty">No stream has produced a message yet.</td>`
        body.append(row)
    }
}

const renderHealth = (health, warning) => {
    element("cpu-busy").textContent = `${(health.cpu_busy * 100).toFixed(0)}%`
    element("load-one").textContent = health.load_one_minute.toFixed(2)
    element("memory").textContent = health.memory_total_bytes > 0
        ? `${bytesToText(health.memory_used_bytes)} / ${bytesToText(health.memory_total_bytes)}`
        : "--"
    element("temperature").textContent = health.temperature_celsius === null
        ? "--"
        : `${health.temperature_celsius.toFixed(1)} C`
    element("disk-free").textContent = health.disk_free_bytes === null
        ? "--"
        : bytesToText(health.disk_free_bytes)

    const cores = element("cpu-cores")
    if (cores.children.length !== health.cpu_cores.length) {
        cores.textContent = ""
        for (const _core of health.cpu_cores) {
            const bar = document.createElement("div")
            bar.className = "core"
            bar.innerHTML = `<span></span>`
            cores.append(bar)
        }
    }
    health.cpu_cores.forEach((busy, index) => {
        const fill = cores.children[index].firstElementChild
        fill.style.width = `${Math.min(100, busy * 100).toFixed(0)}%`
        fill.classList.toggle("hot", busy > 0.85)
    })

    const banner = element("throttle-warning")
    banner.hidden = !warning
    banner.textContent = warning || ""
}

const renderUrdf = (payload) => {
    const report = payload.report
    element("urdf-name").textContent = report.robot_name || "none"
    element("urdf-links").textContent = report.links.length
    element("urdf-joints").textContent = report.joints
    const banner = element("urdf-warning")
    banner.hidden = !payload.warning
    banner.textContent = payload.warning || ""
    const problems = element("urdf-problems")
    problems.textContent = ""
    for (const message of payload.problems || []) {
        const item = document.createElement("li")
        item.textContent = message
        problems.append(item)
    }
}

const renderPreviewTopics = (topics, selected) => {
    const select = element("preview-topic")
    const same = topics.length === select.options.length
        && topics.every((topic, index) => select.options[index].value === topic)
    if (!same) {
        select.textContent = ""
        for (const topic of topics) {
            const option = document.createElement("option")
            option.value = topic
            option.textContent = topic
            select.append(option)
        }
    }
    if (selected && topics.includes(selected)) {
        select.value = selected
    }
}

const applyStatus = (payload) => {
    state.settings = payload.settings
    state.previewTopics = payload.preview_topics
    buildCameraSettings()
    fillSettingControls()
    renderSensors(payload.sensors)
    renderRecording(payload.recording)
    renderStreams(payload.streams)
    renderPreviewTopics(payload.preview_topics, payload.settings.preview_topic)
    renderUrdf(payload.urdf)
    element("preview-enabled").checked = payload.settings.preview_enabled
    element("password-state").textContent = payload.has_password ? "held" : "not set"
    element("password-state").classList.toggle("pill-good", payload.has_password)
    element("removable-mounts").textContent = payload.removable_mounts.length > 0
        ? `Mounted: ${payload.removable_mounts.join(", ")}`
        : "No removable drive mounted."
    if (payload.urdf.report.present && state.urdfXml === null) {
        state.urdfXml = "present"
        loadUrdfViewer()
    }
}

const refreshStatus = async () => {
    applyStatus(await request("/api/status"))
}

// -- sockets --------------------------------------------------------------

const socketUrl = (path) => {
    const scheme = location.protocol === "https:" ? "wss" : "ws"
    return `${scheme}://${location.host}${path}`
}

/** Reconnects forever; a handheld rig loses wifi constantly. */
const keepSocketOpen = (path, onMessage, onState) => {
    let socket = null
    const connect = () => {
        socket = new WebSocket(socketUrl(path))
        socket.binaryType = "blob"
        socket.addEventListener("open", () => onState && onState(true))
        socket.addEventListener("message", onMessage)
        socket.addEventListener("close", () => {
            onState && onState(false)
            setTimeout(connect, 1000)
        })
        socket.addEventListener("error", () => socket.close())
    }
    connect()
}

const startMonitorSocket = () => {
    keepSocketOpen(
        "/ws/monitor",
        (event) => {
            const payload = JSON.parse(event.data)
            renderHealth(payload.health, payload.warning)
            renderStreams(payload.streams)
            renderRecording(payload.recording)
            renderSensors(payload.sensors)
        },
        (open) => {
            element("connection-dot").classList.toggle("dot-good", open)
        },
    )
}

const startPreviewSocket = () => {
    const image = element("preview-image")
    keepSocketOpen("/ws/preview", (event) => {
        if (typeof event.data === "string") {
            return
        }
        const url = URL.createObjectURL(event.data)
        // The previous frame's URL would leak the whole decoded image otherwise,
        // which on a long run is hundreds of megabytes.
        if (state.previewUrl) {
            URL.revokeObjectURL(state.previewUrl)
        }
        state.previewUrl = url
        image.src = url
        image.hidden = false
        element("preview-idle").hidden = true
    })
}

// -- urdf viewer ----------------------------------------------------------

/**
 * A deliberately plain sanity check: one box per link, placed by walking the
 * joint origins. It answers "is my tree the shape I think it is", which is what
 * catches a bad prefix before a recording is made.
 */
const loadUrdfViewer = async () => {
    const host = element("urdf-viewer")
    host.textContent = "loading viewer..."
    let three = null
    try {
        three = await import("https://esm.sh/three@0.180.0")
    } catch (error) {
        host.textContent = "3D viewer unavailable (no internet on this device)."
        return
    }
    let xml = ""
    try {
        const response = await fetch("/api/urdf")
        if (!response.ok) {
            host.textContent = "No URDF uploaded."
            return
        }
        xml = await response.text()
    } catch (error) {
        host.textContent = "No URDF uploaded."
        return
    }

    host.textContent = ""
    const width = host.clientWidth || 480
    const height = 280
    const scene = new three.Scene()
    scene.background = new three.Color(0x11141a)
    const camera = new three.PerspectiveCamera(55, width / height, 0.01, 100)
    camera.position.set(0.9, 0.7, 0.9)
    camera.lookAt(0, 0, 0)
    const renderer = new three.WebGLRenderer({ antialias: true })
    renderer.setSize(width, height)
    host.append(renderer.domElement)
    scene.add(new three.AmbientLight(0xffffff, 0.75))
    const sun = new three.DirectionalLight(0xffffff, 0.9)
    sun.position.set(1, 2, 1)
    scene.add(sun)
    scene.add(new three.GridHelper(2, 20, 0x333a45, 0x222831))

    const document_ = new DOMParser().parseFromString(xml, "application/xml")
    const placements = new Map([["", new three.Vector3(0, 0, 0)]])
    const joints = [...document_.querySelectorAll("joint")]
    const parentOf = new Map()
    const offsetOf = new Map()
    for (const joint of joints) {
        const parent = joint.querySelector("parent")?.getAttribute("link") || ""
        const child = joint.querySelector("child")?.getAttribute("link") || ""
        const origin = joint.querySelector("origin")?.getAttribute("xyz") || "0 0 0"
        const [x, y, z] = origin.trim().split(/\s+/).map(Number)
        parentOf.set(child, parent)
        offsetOf.set(child, new three.Vector3(x || 0, y || 0, z || 0))
    }

    const positionOf = (link, seen) => {
        if (placements.has(link)) {
            return placements.get(link)
        }
        if (seen.has(link)) {
            return new three.Vector3(0, 0, 0)
        }
        seen.add(link)
        const parent = parentOf.get(link)
        const base = parent === undefined
            ? new three.Vector3(0, 0, 0)
            : positionOf(parent, seen).clone()
        const here = base.add(offsetOf.get(link) || new three.Vector3(0, 0, 0))
        placements.set(link, here)
        return here
    }

    const links = [...document_.querySelectorAll("robot > link")]
        .map((link) => link.getAttribute("name"))
        .filter(Boolean)
    for (const link of links) {
        // URDF is z-up and three.js is y-up, so the axes are swapped on the way in.
        const spot = positionOf(link, new Set())
        const box = new three.Mesh(
            new three.BoxGeometry(0.05, 0.05, 0.05),
            new three.MeshStandardMaterial({ color: 0x4c9be8 }),
        )
        box.position.set(spot.x, spot.z, -spot.y)
        scene.add(box)
    }
    for (const [child, parent] of parentOf.entries()) {
        const from = positionOf(parent, new Set())
        const to = positionOf(child, new Set())
        const line = new three.Line(
            new three.BufferGeometry().setFromPoints([
                new three.Vector3(from.x, from.z, -from.y),
                new three.Vector3(to.x, to.z, -to.y),
            ]),
            new three.LineBasicMaterial({ color: 0x8899aa }),
        )
        scene.add(line)
    }

    let angle = 0
    const spin = () => {
        angle = angle + 0.004
        camera.position.set(Math.cos(angle) * 1.2, 0.7, Math.sin(angle) * 1.2)
        camera.lookAt(0, 0.1, 0)
        renderer.render(scene, camera)
        requestAnimationFrame(spin)
    }
    spin()
}

// -- recordings -----------------------------------------------------------

const refreshRecordings = async () => {
    const files = await request("/api/recordings")
    const body = element("recording-rows")
    body.textContent = ""
    for (const file of files) {
        const row = document.createElement("tr")
        const name = document.createElement("td")
        name.textContent = file.name
        const size = document.createElement("td")
        size.textContent = bytesToText(file.bytes)
        const actions = document.createElement("td")
        const remove = document.createElement("button")
        remove.className = "secondary"
        remove.textContent = "Delete"
        remove.addEventListener("click", async () => {
            try {
                await request(`/api/recordings/${encodeURIComponent(file.name)}`, { method: "DELETE" })
                refreshRecordings()
            } catch (error) {
                toast(error.message, true)
            }
        })
        actions.append(remove)
        row.append(name, size, actions)
        body.append(row)
    }
    if (files.length === 0) {
        body.innerHTML = `<tr><td colspan="3" class="empty">Nothing recorded yet.</td></tr>`
    }
}

// -- wiring ---------------------------------------------------------------

const wire = () => {
    element("record-toggle").addEventListener("click", async () => {
        const button = element("record-toggle")
        button.disabled = true
        try {
            if (state.recording.active) {
                renderRecording(await postJson("/api/record/stop"))
                refreshRecordings()
            } else {
                const name = element("recording-name").value.trim()
                renderRecording(await postJson("/api/record/start", name ? { name } : {}))
            }
        } catch (error) {
            toast(error.message, true)
        } finally {
            button.disabled = false
        }
    })

    element("preview-enabled").addEventListener("change", (event) => {
        state.settings.preview_enabled = event.target.checked
        saveSettings()
    })

    element("preview-topic").addEventListener("change", (event) => {
        state.settings.preview_topic = event.target.value
        saveSettings()
    })

    for (const control of document.querySelectorAll("[data-setting]")) {
        bindSettingControl(control)
    }

    element("urdf-file").addEventListener("change", async (event) => {
        const file = event.target.files[0]
        if (!file) {
            return
        }
        const xml = await file.text()
        try {
            // Inspected first so a tree that would break is reported against the
            // file the operator just chose, not the one already saved.
            renderUrdf(await request("/api/urdf/inspect", { method: "POST", body: xml }))
            renderUrdf(await request("/api/urdf", { method: "PUT", body: xml }))
            state.urdfXml = xml
            loadUrdfViewer()
            refreshStatus()
        } catch (error) {
            toast(error.message, true)
        }
    })

    element("urdf-clear").addEventListener("click", async () => {
        try {
            renderUrdf(await request("/api/urdf", { method: "PUT", body: "" }))
            state.urdfXml = null
            element("urdf-viewer").textContent = "No URDF uploaded."
            refreshStatus()
        } catch (error) {
            toast(error.message, true)
        }
    })

    element("save-password").addEventListener("click", async () => {
        const field = element("sudo-password")
        try {
            const result = await postJson("/api/password", { password: field.value })
            // Cleared from the DOM immediately; it lives in the server's memory now.
            field.value = ""
            element("password-state").textContent = result.has_password ? "held" : "not set"
            element("password-state").classList.toggle("pill-good", result.has_password)
            toast(result.has_password ? "password held in memory" : "password cleared")
        } catch (error) {
            toast(error.message, true)
        }
    })

    element("mount-usb").addEventListener("click", async () => {
        try {
            const outcome = await postJson("/api/usb/mount")
            showPlan(outcome)
            refreshStatus()
        } catch (error) {
            toast(error.message, true)
        }
    })

    element("configure-network").addEventListener("click", async () => {
        try {
            showPlan(await postJson("/api/network/mid360", {}))
        } catch (error) {
            toast(error.message, true)
        }
    })

    element("terminal-run").addEventListener("click", runTerminal)
    element("terminal-line").addEventListener("keydown", (event) => {
        if (event.key === "Enter") {
            runTerminal()
        }
    })
}

const showPlan = (outcome) => {
    const lines = []
    for (const step of outcome.steps || []) {
        const result = step.result
        const mark = result.exit_code === 0 ? "ok" : `exit ${result.exit_code}`
        lines.push(`[${mark}] ${step.reason}  (${result.command})`)
        for (const text of [result.stdout, result.stderr]) {
            if (text && text.trim()) {
                lines.push(text.trim())
            }
        }
    }
    if (outcome.note) {
        lines.push(outcome.note)
    }
    if (outcome.failed) {
        lines.push(`failed: ${outcome.failed}`)
    }
    element("terminal-output").textContent = lines.join("\n")
    element("terminal-card").open = true
    toast(outcome.failed ? `failed: ${outcome.failed}` : "done", Boolean(outcome.failed))
}

const runTerminal = async () => {
    const line = element("terminal-line").value.trim()
    if (!line) {
        return
    }
    const output = element("terminal-output")
    output.textContent = "running..."
    try {
        const result = await postJson("/api/terminal", {
            line,
            as_root: element("terminal-root").checked,
        })
        const parts = []
        if (result.stdout) {
            parts.push(result.stdout)
        }
        if (result.stderr) {
            parts.push(result.stderr)
        }
        parts.push(`exit ${result.exit_code}`)
        output.textContent = parts.join("\n")
    } catch (error) {
        output.textContent = error.message
    }
}

// -- start ----------------------------------------------------------------

const start = async () => {
    wire()
    try {
        await refreshStatus()
        await refreshRecordings()
    } catch (error) {
        toast(error.message, true)
    }
    startMonitorSocket()
    startPreviewSocket()
}

start()

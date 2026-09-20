/**
 * The recorder's browser front end. No build step: plain modules, and three.js
 * is a vendored file the binary serves, so the Pi never runs a bundler and the
 * phone never needs the internet.
 */

const element = (id) => document.getElementById(id)

/** Where the operator password is cached, so one browser is asked once. */
const PASSWORD_KEY = "lite_record_password"

const state = {
    /** @type {object|null} last full status payload from /api/status */
    settings: null,
    recording: { active: false },
    previewTopics: [],
    /** topic -> the settings path that switches it off, from /api/status */
    topicSettings: {},
    /** which camera's stream switches sit under the preview */
    previewCamera: null,
    /** the object URL currently shown, revoked when the next frame lands */
    previewUrl: null,
    urdfXml: null,
    /** set while a settings PUT is in flight, so the poll does not clobber typing */
    savingSettings: false,
    /** the charted series, replaced only on the ticks that carry a new one */
    history: null,
    /** so the memory chart can label a percentage series in bytes */
    memoryTotalBytes: 0,
    /** whether the server has a password at all, and whether ours is right */
    access: { password_required: false, allowed: true },
    /** resolves when the password sheet is answered, so a 401 can retry */
    passwordAsk: null,
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

/** Unix seconds to a local date and time. The Pi's clock is often in another
 * zone from whoever is holding the browser, so the browser does the formatting. */
const timestampToText = (seconds) => {
    if (!seconds) {
        return "unknown"
    }
    const when = new Date(seconds * 1000)
    return `${when.toLocaleDateString()} ${when.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}`
}

const secondsToText = (seconds) => {
    if (!seconds || seconds < 60) {
        return `${(seconds || 0).toFixed(1)} s`
    }
    const minutes = Math.floor(seconds / 60)
    return `${minutes}m ${(seconds - minutes * 60).toFixed(0)}s`
}

const toast = (message, bad) => {
    const box = element("toast")
    box.textContent = message
    box.hidden = false
    box.classList.toggle("toast-bad", Boolean(bad))
    clearTimeout(toast.timer)
    toast.timer = setTimeout(() => { box.hidden = true }, 4000)
}

/** Builds an element in one call, because the alternative is four lines of
 * createElement/className/textContent for every cell in every table. */
const make = (tag, className, text) => {
    const node = document.createElement(tag)
    if (className) {
        node.className = className
    }
    if (text !== undefined) {
        node.textContent = text
    }
    return node
}

// -- requests and the operator password -----------------------------------

const cachedPassword = () => localStorage.getItem(PASSWORD_KEY) || ""

/**
 * Every call goes through here so a backend error message reaches the operator
 * instead of vanishing into the console, and so the operator password rides
 * along on the requests that need it without every call site remembering.
 */
const request = async (path, options, retrying) => {
    const settings = { ...(options || {}) }
    const password = cachedPassword()
    if (password) {
        settings.headers = { ...(settings.headers || {}), "x-lite-record-password": password }
    }
    const response = await fetch(path, settings)
    const text = await response.text()
    let body = null
    if (text) {
        try {
            body = JSON.parse(text)
        } catch (error) {
            body = { error: text }
        }
    }
    if (response.status === 401 && body && body.password_required && !retrying) {
        // The cached one is wrong or missing. Ask once, then replay the request
        // so the press the operator actually made is the one that happens.
        const given = await askForPassword()
        if (given) {
            return request(path, options, true)
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

// -- sheets ---------------------------------------------------------------

/**
 * One sheet, reused. A sheet rather than a new page because every one of these
 * — a summary, a destination, a password — is a detour from something the
 * operator was already doing, and closing it should put them back exactly there.
 */
const openSheet = (title, build) => {
    element("sheet-title").textContent = title
    const body = element("sheet-body")
    body.textContent = ""
    build(body)
    element("scrim").hidden = false
    element("sheet").hidden = false
    document.body.classList.add("sheet-open")
}

const closeSheet = () => {
    element("scrim").hidden = true
    element("sheet").hidden = true
    document.body.classList.remove("sheet-open")
    // A sheet dismissed while it was asking for a password is an answer of "no".
    if (state.passwordAsk) {
        const settle = state.passwordAsk
        state.passwordAsk = null
        settle(null)
    }
}

/**
 * Asks for the operator password and caches it. Resolves with the password when
 * one was given and with null when the operator dismissed the sheet, which is
 * what tells a refused request to give up rather than loop.
 */
const askForPassword = () => new Promise((resolve) => {
    openSheet("Password needed", (body) => {
        body.append(make("p", "hint", "This action runs a command on the recorder. The password is kept in this browser, so you are asked once."))
        const field = document.createElement("input")
        field.type = "password"
        field.autocomplete = "current-password"
        field.placeholder = "operator password"
        const unlock = make("button", "", "Unlock")
        const row = make("div", "row")
        row.append(field, unlock)
        body.append(row)

        const submit = async () => {
            if (!field.value) {
                return
            }
            localStorage.setItem(PASSWORD_KEY, field.value)
            const given = field.value
            state.passwordAsk = null
            await refreshAccess()
            closeSheet()
            resolve(given)
        }
        unlock.addEventListener("click", submit)
        field.addEventListener("keydown", (event) => {
            if (event.key === "Enter") {
                submit()
            }
        })
        setTimeout(() => field.focus(), 50)
    })
    state.passwordAsk = resolve
})

const refreshAccess = async () => {
    try {
        state.access = await request("/api/access")
    } catch (error) {
        return
    }
    renderAccess()
}

const renderAccess = () => {
    const pill = element("access-state")
    const required = state.access.password_required
    const allowed = state.access.allowed
    pill.textContent = !required ? "no password" : allowed ? "unlocked" : "locked"
    pill.classList.toggle("pill-good", required && allowed)
    pill.classList.toggle("pill-bad", required && !allowed)
    element("access-change").textContent = required ? "Change password" : "Set password"
    element("access-forget").hidden = !cachedPassword()
}

const openAccessSheet = () => {
    openSheet(state.access.password_required ? "Change password" : "Set password", (body) => {
        body.append(make("p", "hint", "Guards the terminal, mounting a drive, the lidar network setup and the sudo password. Leave the new password empty to remove it."))
        const fields = {}
        const add = (key, label, placeholder) => {
            const row = make("div", "row")
            row.append(make("label", "", label))
            const field = document.createElement("input")
            field.type = "password"
            field.autocomplete = "new-password"
            field.placeholder = placeholder || ""
            row.append(field)
            body.append(row)
            fields[key] = field
        }
        if (state.access.password_required) {
            add("current", "Current", "the password in force")
        }
        add("next", "New", "at least 6 characters")
        add("again", "Repeat")

        const save = make("button", "", "Save")
        save.addEventListener("click", async () => {
            if (fields.next.value !== fields.again.value) {
                toast("the two new passwords do not match", true)
                return
            }
            const current = fields.current ? fields.current.value : cachedPassword()
            try {
                const result = await postJson("/api/access", {
                    current: current || null,
                    next: fields.next.value || null,
                })
                if (fields.next.value) {
                    localStorage.setItem(PASSWORD_KEY, fields.next.value)
                } else {
                    localStorage.removeItem(PASSWORD_KEY)
                }
                state.access = { password_required: result.password_required, allowed: true }
                renderAccess()
                closeSheet()
                toast(result.password_required ? "password set" : "password removed")
            } catch (error) {
                toast(error.message, true)
            }
        })
        body.append(save)
    })
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
    control.addEventListener("change", () => {
        if (!state.settings) {
            return
        }
        writePath(state.settings, path, readControl(control))
        saveSettings()
    })
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

// -- sensor settings blocks ----------------------------------------------

const cameraStreamFields = [
    { key: "depth", label: "Depth" },
    { key: "color", label: "Colour" },
    { key: "infrared", label: "Infrared" },
    { key: "imu", label: "IMU" },
    { key: "emitter", label: "IR emitter" },
    { key: "align_depth_to_color", label: "Align depth to colour" },
]

const checkbox = (path, text) => {
    const label = make("label", "check")
    const box = document.createElement("input")
    box.type = "checkbox"
    box.dataset.setting = path
    label.append(box, make("span", "", text))
    return label
}

const numberField = (path, text, step) => {
    const label = document.createElement("label")
    const input = document.createElement("input")
    input.type = "number"
    input.min = "0"
    if (step) {
        input.step = step
    }
    input.dataset.setting = path
    label.append(document.createTextNode(text), input)
    return label
}

const textField = (path, text, placeholder) => {
    const label = document.createElement("label")
    const input = document.createElement("input")
    input.type = "text"
    input.dataset.setting = path
    if (placeholder) {
        input.placeholder = placeholder
    }
    label.append(document.createTextNode(text), input)
    return label
}

const pair = (...children) => {
    const host = make("div", "field-pair")
    host.append(...children)
    return host
}

/**
 * One collapsed block per sensor. `Enabled` sits in the header because it is
 * the only control touched often; everything else — resolutions, prefixes,
 * addresses — is behind the disclosure, which is what keeps this tab from
 * being four screens of checkboxes.
 */
const buildSensorSettings = () => {
    const host = element("sensor-settings")
    if (host.dataset.built) {
        return
    }
    host.dataset.built = "yes"

    for (const kind of ["realsense", "orbbec", "oakd", "livox"]) {
        const block = make("div", "config")
        block.dataset.kind = kind

        const head = make("div", "config-head")
        head.append(checkbox(`${kind}.enabled`, ""))
        const title = make("div", "config-title")
        title.append(make("b", "", sensorTitles[kind]))
        title.append(make("span", "config-summary", ""))
        head.append(title)

        const disclose = make("button", "config-toggle secondary", "")
        disclose.type = "button"
        disclose.setAttribute("aria-expanded", "false")
        disclose.setAttribute("aria-label", `${sensorTitles[kind]} settings`)
        head.append(disclose)
        block.append(head)

        const body = make("div", "config-body")
        body.hidden = true
        body.append(kind === "livox" ? livoxFields() : cameraFields(kind))
        block.append(body)

        const flip = () => {
            const open = body.hidden
            body.hidden = !open
            block.classList.toggle("open", open)
            disclose.setAttribute("aria-expanded", String(open))
        }
        disclose.addEventListener("click", flip)
        title.addEventListener("click", flip)

        host.append(block)
    }

    for (const control of host.querySelectorAll("[data-setting]")) {
        bindSettingControl(control)
    }
}

const sensorTitles = {
    realsense: "RealSense",
    orbbec: "Orbbec",
    oakd: "OAK-D",
    livox: "Livox Mid-360",
}

const cameraFields = (kind) => {
    const host = document.createDocumentFragment()

    const streams = make("div", "row wrap")
    for (const field of cameraStreamFields) {
        streams.append(checkbox(`${kind}.${field.key}`, field.label))
    }
    host.append(streams)

    host.append(pair(
        numberField(`${kind}.width`, "Width"),
        numberField(`${kind}.height`, "Height"),
    ))
    host.append(pair(
        numberField(`${kind}.frame_rate`, "Frame rate"),
        numberField(`${kind}.imu_rate`, "IMU rate (Hz)"),
    ))

    const advanced = make("details", "note")
    advanced.append(make("summary", "", "Naming and device"))
    advanced.append(pair(
        textField(`${kind}.naming.topic_prefix`, "Topic prefix"),
        textField(`${kind}.naming.frame_prefix`, "Frame prefix"),
    ))
    advanced.append(pair(textField(`${kind}.serial`, "Serial", "any")))
    host.append(advanced)

    return host
}

const livoxFields = () => {
    const host = document.createDocumentFragment()

    const toggles = make("div", "row wrap")
    toggles.append(checkbox("livox.imu", "IMU (200 Hz)"))
    host.append(toggles)

    host.append(pair(
        numberField("livox.frame_hz", "Cloud Hz", "0.5"),
        numberField("livox.voxel_leaf_size", "Voxel leaf (m)", "0.01"),
    ))
    host.append(make("p", "hint", "A leaf size of 0 keeps every point. Downsampling happens before encoding, so it saves both CPU and disk."))

    const advanced = make("details", "note")
    advanced.append(make("summary", "", "Naming and network"))
    advanced.append(pair(
        textField("livox.naming.topic_prefix", "Topic prefix"),
        textField("livox.naming.frame_prefix", "Frame prefix"),
    ))
    advanced.append(pair(
        textField("livox.host_address", "Host address", "auto"),
        textField("livox.lidar_address", "Lidar address", "multicast"),
    ))

    const nic = make("div", "field")
    const label = document.createElement("label")
    label.htmlFor = "lidar-interface"
    label.textContent = "Lidar NIC"
    const input = document.createElement("input")
    input.id = "lidar-interface"
    input.type = "text"
    input.placeholder = "eth0"
    input.autocomplete = "off"
    nic.append(label, input)
    advanced.append(nic)

    // Wired here rather than in `wire`, which runs before this block exists.
    const configure = make("button", "secondary", "Auto-configure network")
    configure.type = "button"
    configure.addEventListener("click", async () => {
        const interfaceName = input.value.trim()
        if (!interfaceName) {
            toast("name the interface the lidar is plugged into, such as eth0", true)
            return
        }
        try {
            showPlan(await postJson("/api/network/mid360", {
                interface: interfaceName,
                host_address: state.settings.livox.host_address,
            }))
        } catch (error) {
            toast(error.message, true)
        }
    })
    advanced.append(configure)

    host.append(advanced)
    return host
}

/**
 * The one-line state under each sensor's name. It exists so the collapsed list
 * still answers "what is this set to" without four taps.
 */
const renderSensorSummaries = () => {
    for (const block of document.querySelectorAll("#sensor-settings .config")) {
        const kind = block.dataset.kind
        const config = state.settings?.[kind]
        const line = block.querySelector(".config-summary")
        if (!config) {
            line.textContent = ""
            continue
        }
        if (kind === "livox") {
            line.textContent = `${config.frame_hz} Hz${config.imu ? " · IMU" : ""}`
            continue
        }
        const streams = cameraStreamFields
            .filter((field) => field.key !== "emitter" && field.key !== "align_depth_to_color")
            .filter((field) => config[field.key])
            .map((field) => field.label.toLowerCase())
        line.textContent = `${config.width}x${config.height} @ ${config.frame_rate} · ${streams.join(", ") || "no streams"}`
    }
}

// -- rendering ------------------------------------------------------------

const renderSensors = (sensors) => {
    const host = element("sensor-list")
    host.textContent = ""
    for (const [kind, status] of Object.entries(sensors)) {
        const row = make("div", "sensor")
        row.dataset.sensor = kind

        const name = make("strong", "", kind)
        const detail = make("span", "detail", status.error ? status.error : status.detail)
        detail.classList.toggle("bad", Boolean(status.error))

        const pill = make("span", `pill ${status.running ? "pill-good" : "pill-idle"}`,
            status.running ? "engaged" : "disengaged")
        pill.dataset.role = "state"

        const actions = make("span", "sensor-actions")
        // Restart is offered on top of the settings being applied automatically,
        // because a camera that has wedged needs a cycle with nothing changed.
        const buttons = status.running
            ? [["Restart", "restart"], ["Disengage", "disengage"]]
            : [["Engage", "engage"]]
        for (const [label, action] of buttons) {
            const button = make("button", "secondary", label)
            button.dataset.role = action
            button.addEventListener("click", async () => {
                button.disabled = true
                try {
                    renderSensors(await postJson(`/api/sensors/${kind}/${action}`))
                } catch (error) {
                    toast(error.message, true)
                    button.disabled = false
                }
            })
            actions.append(button)
        }

        row.append(name, pill, detail, actions)
        host.append(row)
    }
}

const renderRecording = (recording) => {
    // The list below is not polled, so this is what puts a finished recording in
    // it. Doing it here rather than in the stop handler covers every way a
    // recording can end: another browser stopping it, the disk filling, or the
    // stop request itself timing out while the multi-gigabyte file finalises.
    const wasActive = state.recording.active
    state.recording = recording
    if (wasActive && !recording.active) {
        refreshRecordings()
    }
    element("recording-state").textContent = recording.active ? "recording" : "idle"
    element("recording-state").classList.toggle("good", recording.active)
    element("recording-seconds").textContent = secondsToText(recording.seconds)
    element("recording-messages").textContent = recording.messages.toLocaleString()
    element("recording-bytes").textContent = bytesToText(recording.bytes)
    element("recording-dropped").textContent = recording.dropped.toLocaleString()
    element("recording-dropped").classList.toggle("bad", recording.dropped > 0)
    element("recording-path").textContent = recording.path || ""
    const toggle = element("record-toggle")
    toggle.textContent = recording.active ? "Stop recording" : "Start recording"
    toggle.classList.toggle("recording", recording.active)
    document.body.classList.toggle("is-recording", recording.active)
    // The elapsed time follows into the header, so it stays visible from the
    // Monitor and Files tabs while a recording runs.
    const clock = element("record-clock")
    clock.hidden = !recording.active
    clock.textContent = secondsToText(recording.seconds)
}

/**
 * Rows are reused rather than rebuilt, because this runs on every monitor tick
 * and a checkbox replaced ten times a second cannot be clicked.
 */
const streamRow = (topic) => {
    const body = element("stream-rows")
    const existing = body.querySelector(`tr[data-topic="${CSS.escape(topic)}"]`)
    if (existing) {
        return existing
    }
    const row = document.createElement("tr")
    row.dataset.topic = topic
    row.innerHTML = `<td class="topic"></td><td></td><td></td><td></td><td class="switch"></td>`
    row.firstChild.textContent = topic
    const path = state.topicSettings[topic]
    if (path) {
        const label = make("label", "check")
        label.innerHTML = `<input type="checkbox" data-setting="${path}"><span class="sr-only">on</span>`
        row.lastChild.append(label)
        bindSettingControl(label.firstChild)
        label.firstChild.checked = Boolean(readPath(state.settings, path))
    }
    body.append(row)
    return row
}

const renderStreams = (streams) => {
    const body = element("stream-rows")
    if (streams.length === 0) {
        if (!body.querySelector(".empty")) {
            body.innerHTML = `<tr><td colspan="5" class="empty">No stream has produced a message yet.</td></tr>`
        }
        return
    }
    // A row caches its toggle, so rows built before the topic map arrived — or
    // before a prefix was renamed — have to be thrown away rather than reused.
    if (body.builtFor !== state.topicSettings) {
        body.textContent = ""
        body.builtFor = state.topicSettings
    }
    const wanted = new Set(streams.map((stream) => stream.topic))
    for (const row of [...body.children]) {
        if (!wanted.has(row.dataset.topic)) {
            row.remove()
        }
    }
    for (const stream of streams) {
        const row = streamRow(stream.topic)
        const [, hz, total, dropped] = row.children
        hz.textContent = stream.hz.toFixed(1)
        total.textContent = stream.total.toLocaleString()
        dropped.textContent = stream.dropped.toLocaleString()
        dropped.classList.toggle("bad", stream.dropped > 0)
    }
}

// -- charts ---------------------------------------------------------------

const CHART_WIDTH = 240
const CHART_HEIGHT = 64

/**
 * A line and the area under it, drawn straight into the SVG that is already in
 * the page. No chart library: the Pi is usually off the internet, so anything
 * fetched from a CDN would leave the operator staring at an empty box exactly
 * when the machine is in the field.
 *
 * `low` and `high` are the axis, given rather than derived, because a chart that
 * rescales itself every second makes a flat trace look like a mountain range —
 * which is the whole complaint the charts are here to answer.
 */
const drawChart = (figure, series, low, high, format) => {
    const line = figure.querySelector(".chart-line")
    const area = figure.querySelector(".chart-area")
    const span = figure.querySelector(".chart-span")
    const top = figure.querySelector(".chart-top")
    if (!series || series.length < 2) {
        line.removeAttribute("d")
        area.removeAttribute("d")
        span.textContent = ""
        top.textContent = "collecting..."
        return
    }
    const range = Math.max(high - low, 1e-6)
    const point = (value, index) => {
        const x = (index / (series.length - 1)) * CHART_WIDTH
        const y = CHART_HEIGHT - ((value - low) / range) * (CHART_HEIGHT - 4) - 2
        return `${x.toFixed(2)} ${Math.max(0, Math.min(CHART_HEIGHT, y)).toFixed(2)}`
    }
    const path = series.map((value, index) => `${index === 0 ? "M" : "L"} ${point(value, index)}`).join(" ")
    line.setAttribute("d", path)
    area.setAttribute("d", `${path} L ${CHART_WIDTH} ${CHART_HEIGHT} L 0 ${CHART_HEIGHT} Z`)

    const seconds = series.length * (state.history ? state.history.interval_seconds : 1)
    span.textContent = seconds >= 90
        ? `last ${Math.round(seconds / 60)} min`
        : `last ${Math.round(seconds)} s`
    top.textContent = `peak ${format(Math.max(...series))}`
}

const percentText = (value) => `${value.toFixed(0)}%`
const celsiusText = (value) => `${value.toFixed(1)}°C`

const renderCharts = () => {
    const history = state.history
    if (!history) {
        return
    }
    const figure = (name) => document.querySelector(`.chart[data-chart="${name}"]`)
    drawChart(figure("cpu"), history.cpu_percent, 0, 100, percentText)
    // The memory headline is absolute, so its peak is too — "389 MB used, peak
    // 10%" reads like two different measurements of two different things.
    const memoryText = state.memoryTotalBytes
        ? (percent) => bytesToText((percent / 100) * state.memoryTotalBytes)
        : percentText
    drawChart(figure("memory"), history.memory_percent, 0, 100, memoryText)

    // Temperature never approaches zero and the interesting band is narrow, so
    // a 0..100 axis would draw a Pi climbing from 45 to 80 degrees as a flat
    // line. The axis is snapped to ten-degree steps so it stops jumping about
    // as the extremes drift.
    const temperatures = history.temperature_celsius
    if (temperatures && temperatures.length > 0) {
        const low = Math.floor(Math.min(...temperatures) / 10) * 10
        const high = Math.max(low + 20, Math.ceil(Math.max(...temperatures) / 10) * 10)
        drawChart(figure("temperature"), temperatures, low, high, celsiusText)
    }
}

const renderHealth = (health, warning) => {
    state.memoryTotalBytes = health.memory_total_bytes
    element("cpu-busy").textContent = percentText(health.cpu_busy * 100)
    element("load-one").textContent = health.load_one_minute.toFixed(2)
    element("memory").textContent = health.memory_total_bytes > 0
        ? `${bytesToText(health.memory_used_bytes)} / ${bytesToText(health.memory_total_bytes)}`
        : "--"
    element("temperature").textContent = health.temperature_celsius === null
        ? "--"
        : celsiusText(health.temperature_celsius)
    element("disk-free").textContent = health.disk_free_bytes === null
        ? "--"
        : bytesToText(health.disk_free_bytes)

    const cores = element("cpu-cores")
    if (cores.children.length !== health.cpu_cores.length) {
        cores.textContent = ""
        health.cpu_cores.forEach((_busy, index) => {
            const core = make("div", "core")
            // Named and numbered: an unlabelled bar is a bar whose meaning has
            // to be guessed, which is exactly the complaint.
            core.innerHTML = `<span class="core-name">cpu${index}</span>`
                + `<span class="core-track"><span class="core-fill"></span></span>`
                + `<span class="core-value">--</span>`
            cores.append(core)
        })
    }
    health.cpu_cores.forEach((busy, index) => {
        const core = cores.children[index]
        const fill = core.querySelector(".core-fill")
        fill.style.width = `${Math.min(100, busy * 100).toFixed(0)}%`
        fill.classList.toggle("hot", busy > 0.85)
        core.querySelector(".core-value").textContent = percentText(busy * 100)
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
    // The same line above the fold, so a broken tree is seen before recording,
    // not found in the settings tab afterwards.
    const global = element("tf-warning")
    global.hidden = !payload.warning
    global.textContent = payload.warning || ""
    const problems = element("urdf-problems")
    problems.textContent = ""
    for (const message of payload.problems || []) {
        problems.append(make("li", "", message))
    }
}

/** True when the stream behind a topic is switched on in settings. */
const streamIsOn = (topic) => {
    const path = state.topicSettings[topic]
    return path ? Boolean(readPath(state.settings, path)) : true
}

const renderPreviewTopics = (topics, selected) => {
    const select = element("preview-topic")
    // Switched-off streams are listed so they can be switched back on, but they
    // are labelled, because otherwise picking one just shows an empty frame.
    const labels = topics.map((topic) => (streamIsOn(topic) ? topic : `${topic} (off)`))
    const same = labels.length === select.options.length
        && labels.every((label, index) => select.options[index].textContent === label)
    if (!same) {
        select.textContent = ""
        for (const [index, topic] of topics.entries()) {
            const option = document.createElement("option")
            option.value = topic
            option.textContent = labels[index]
            select.append(option)
        }
    }
    if (selected && topics.includes(selected)) {
        select.value = selected
    }
    renderPreviewStreams(selected || select.value)
}

/**
 * The stream switches for whichever camera is being previewed, sitting next to
 * the picture of it. The camera comes from the topic's settings path rather than
 * from the topic text, so a renamed prefix still finds it.
 */
const renderPreviewStreams = (topic) => {
    const host = element("preview-streams")
    const path = state.topicSettings[topic]
    const kind = path ? path.split(".")[0] : null
    if (!kind || !state.settings || !state.settings[kind]) {
        host.hidden = true
        return
    }
    host.hidden = false
    if (state.previewCamera !== kind) {
        state.previewCamera = kind
        host.textContent = ""
        host.append(`${kind} streams`)
        for (const field of ["depth", "color", "infrared", "imu"]) {
            const label = make("label", "check")
            label.innerHTML = `<input type="checkbox" data-setting="${kind}.${field}"><span>${field}</span>`
            bindSettingControl(label.firstChild)
            host.append(label)
        }
    }
    for (const control of host.querySelectorAll("[data-setting]")) {
        control.checked = Boolean(readPath(state.settings, control.dataset.setting))
    }
}

const applyStatus = (payload) => {
    state.settings = payload.settings
    state.previewTopics = payload.preview_topics
    state.topicSettings = payload.topic_settings
    buildSensorSettings()
    fillSettingControls()
    renderSensorSummaries()
    renderRecordDir()
    renderSensors(payload.sensors)
    renderRecording(payload.recording)
    renderStreams(payload.streams)
    renderPreviewTopics(payload.preview_topics, payload.preview_topic)
    renderUrdf(payload.urdf)
    if (payload.tf_warning && !payload.urdf.warning) {
        const global = element("tf-warning")
        global.hidden = false
        global.textContent = payload.tf_warning
    }
    element("preview-enabled").checked = payload.settings.preview_enabled
    element("password-state").textContent = payload.has_password ? "held" : "not set"
    element("password-state").classList.toggle("pill-good", payload.has_password)
    element("removable-mounts").textContent = payload.removable_mounts.length > 0
        ? `Mounted: ${payload.removable_mounts.join(", ")}`
        : "No removable drive mounted."
    state.access.password_required = payload.password_required
    renderAccess()
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
            // Sent only on the tick it changed, so the other four ticks a second
            // carry no series at all and there is nothing to redraw.
            if (payload.history) {
                state.history = payload.history
                renderCharts()
            }
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

// -- lidar view -----------------------------------------------------------

/**
 * The live scan as points, thinned on the Pi (see `/ws/cloud`). Each frame is
 * a flat little-endian f32 array of x y z intensity, handed to one buffer
 * that is reused rather than rebuilt. The lidar is z-up and three.js is y-up,
 * so the axes are swapped on the way in, the same as the URDF viewer.
 */
const loadCloudViewer = async (canvas) => {
    const three = await import("three")
    const { OrbitControls } = await import("three/addons/OrbitControls.js")
    const frame = canvas.parentElement
    const renderer = new three.WebGLRenderer({ canvas, antialias: false })
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
    const scene = new three.Scene()
    scene.background = new three.Color(0x000000)
    const camera = new three.PerspectiveCamera(55, 1, 0.05, 200)
    camera.position.set(0, 4, 8)
    const controls = new OrbitControls(camera, canvas)
    controls.enableDamping = true
    controls.target.set(0, 0, 0)
    scene.add(new three.GridHelper(20, 20, 0x333a45, 0x222831))
    scene.add(new three.AxesHelper(0.5))

    const capacity = 8192
    const positions = new Float32Array(capacity * 3)
    const colors = new Float32Array(capacity * 3)
    const geometry = new three.BufferGeometry()
    geometry.setAttribute("position", new three.BufferAttribute(positions, 3).setUsage(three.DynamicDrawUsage))
    geometry.setAttribute("color", new three.BufferAttribute(colors, 3).setUsage(three.DynamicDrawUsage))
    geometry.setDrawRange(0, 0)
    const material = new three.PointsMaterial({ size: 0.04, vertexColors: true })
    const points = new three.Points(geometry, material)
    points.frustumCulled = false
    scene.add(points)

    const resize = () => {
        const width = frame.clientWidth || 320
        const height = frame.clientHeight || 320
        renderer.setSize(width, height, false)
        camera.aspect = width / height
        camera.updateProjectionMatrix()
    }
    new ResizeObserver(resize).observe(frame)
    resize()

    // Floor dark blue, head height yellow, above that white.
    const shade = (z, out) => {
        const t = Math.min(Math.max((z + 1) / 3, 0), 1)
        out[0] = 0.15 + 0.85 * t
        out[1] = 0.25 + 0.6 * t
        out[2] = 0.9 - 0.7 * t
    }

    let paused = false
    let dirty = true
    const draw = () => {
        requestAnimationFrame(draw)
        if (paused) {
            return
        }
        controls.update()
        if (dirty || controls.enableDamping) {
            renderer.render(scene, camera)
            dirty = false
        }
    }
    draw()

    const rgb = [0, 0, 0]
    return {
        update(floats) {
            const count = Math.min(Math.floor(floats.length / 4), capacity)
            for (let i = 0; i < count; i++) {
                const x = floats[i * 4]
                const y = floats[i * 4 + 1]
                const z = floats[i * 4 + 2]
                positions[i * 3] = x
                positions[i * 3 + 1] = z
                positions[i * 3 + 2] = -y
                shade(z, rgb)
                colors[i * 3] = rgb[0]
                colors[i * 3 + 1] = rgb[1]
                colors[i * 3 + 2] = rgb[2]
            }
            geometry.setDrawRange(0, count)
            geometry.attributes.position.needsUpdate = true
            geometry.attributes.color.needsUpdate = true
            dirty = true
            return count
        },
        setPaused(value) {
            paused = value
        },
    }
}

/**
 * Off by default and off whenever the page is hidden: the socket is what
 * makes the Pi thin scans, so closing it is what saves the work.
 */
const startCloudView = () => {
    const toggle = element("cloud-enabled")
    const canvas = element("cloud-canvas")
    const idle = element("cloud-idle")
    const count = element("cloud-points")
    let viewer = null
    let socket = null

    const disconnect = () => {
        if (socket) {
            const closing = socket
            socket = null
            closing.close()
        }
    }
    const connect = () => {
        if (socket || !toggle.checked || document.hidden) {
            return
        }
        const opened = new WebSocket(socketUrl("/ws/cloud"))
        opened.binaryType = "arraybuffer"
        socket = opened
        opened.addEventListener("message", (event) => {
            if (typeof event.data === "string") {
                return
            }
            const shown = viewer.update(new Float32Array(event.data))
            count.textContent = `${shown} points`
            canvas.hidden = false
            idle.hidden = true
        })
        opened.addEventListener("close", () => {
            if (socket === opened) {
                socket = null
                setTimeout(connect, 1000)
            }
        })
        opened.addEventListener("error", () => opened.close())
    }
    const stop = () => {
        disconnect()
        canvas.hidden = true
        idle.hidden = false
        idle.textContent = "Off."
        count.textContent = ""
        if (viewer) {
            viewer.setPaused(true)
        }
    }
    const start = async () => {
        if (!viewer) {
            idle.textContent = "Loading the viewer..."
            try {
                viewer = await loadCloudViewer(canvas)
            } catch (error) {
                toast(`could not load the 3D view: ${error.message}`, true)
                toggle.checked = false
                idle.textContent = "Off."
                return
            }
        }
        viewer.setPaused(false)
        idle.textContent = "Waiting for a scan..."
        connect()
    }

    toggle.addEventListener("change", () => (toggle.checked ? start() : stop()))
    document.addEventListener("visibilitychange", () => {
        if (document.hidden) {
            disconnect()
            if (viewer) {
                viewer.setPaused(true)
            }
        } else if (toggle.checked) {
            if (viewer) {
                viewer.setPaused(false)
            }
            connect()
        }
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
        three = await import("three")
    } catch (error) {
        host.textContent = "3D viewer unavailable."
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

// -- the recording summary sheet ------------------------------------------

/**
 * What `db_summary` prints for a `.db`, for an mcap. The server reads the file's
 * own message indexes rather than its payloads, so this comes back in a fraction
 * of a second even for a file of several gigabytes.
 */
const openSummarySheet = (file) => {
    openSheet(file.name, (body) => {
        body.append(make("p", "hint", "Reading the index..."))
        request(`/api/recordings/${encodeURIComponent(file.name)}/summary`).then((summary) => {
            body.textContent = ""
            const stats = make("dl", "stats")
            const pairs = [
                ["Recorded", timestampToText(summary.start_unix_seconds)],
                ["Duration", secondsToText(summary.duration_seconds)],
                ["Messages", summary.message_count.toLocaleString()],
                ["Topics", String(summary.topics.length)],
                ["Size", bytesToText(summary.file_bytes)],
            ]
            for (const [term, value] of pairs) {
                const pair = document.createElement("div")
                pair.append(make("dt", "", term), make("dd", "", value))
                stats.append(pair)
            }
            body.append(stats)

            if (!summary.indexed) {
                body.append(make("p", "hint", "This file has no message index, so it was read in full."))
            }

            const table = make("table", "streams summary-table")
            table.innerHTML = `<thead><tr>
                <th>Topic</th><th>Type</th><th>Count</th><th>Secs</th><th>Hz</th>
                <th>p99 gap</th><th>Worst gap</th><th>Size</th>
            </tr></thead>`
            const rows = document.createElement("tbody")
            for (const topic of summary.topics) {
                const row = document.createElement("tr")
                // The multiple of the stream's own average is what says whether a
                // gap is a stall or just how that stream behaves.
                const worst = `${topic.worst_gap_seconds.toFixed(3)}s (${topic.worst_gap_ratio.toFixed(1)}x)`
                for (const [text, className] of [
                    [topic.topic, "topic"],
                    [topic.payload.split("/").pop(), ""],
                    [topic.count.toLocaleString(), ""],
                    [topic.duration_seconds.toFixed(1), ""],
                    [topic.hz.toFixed(2), ""],
                    [`${topic.p99_gap_seconds.toFixed(4)}s`, ""],
                    [worst, topic.worst_gap_ratio >= 10 ? "bad" : ""],
                    [bytesToText(topic.bytes), ""],
                ]) {
                    row.append(make("td", className, text))
                }
                rows.append(row)
            }
            table.append(rows)
            const scroller = make("div", "table-scroll")
            scroller.append(table)
            body.append(scroller)

            if (summary.frames.length > 0) {
                body.append(make("h3", "", "Frames in /tf_static"))
                const list = make("ul", "frame-list")
                for (const frame of summary.frames) {
                    list.append(make("li", "", `${frame.parent} → ${frame.child}`))
                }
                body.append(list)
            }
        }).catch((error) => {
            body.textContent = ""
            body.append(make("p", "warning", error.message))
        })
    })
}

// -- the move sheet -------------------------------------------------------

/**
 * Pick a mounted volume, then a directory on it, then move. The listing comes
 * from the server on every step rather than being cached, because a USB stick
 * can be pulled out between one screen and the next.
 */
// -- folder picker --------------------------------------------------------

/**
 * Drive list, then a walk down the directories on it. Used for both "move this
 * recording somewhere" and "record into this folder", which is why it takes its
 * wording and its confirm action from the caller rather than being written
 * twice and drifting.
 *
 * `needBytes` greys out a drive that cannot hold what is going onto it, and
 * `start` opens straight into a folder instead of the drive list, which is what
 * makes changing an already-set path a one-tap edit rather than a re-navigation.
 */
const openFolderPicker = ({ title, lead, confirm, needBytes = 0, start = null, onPick }) => {
    openSheet(title, (body) => {
        const showVolumes = async () => {
            body.textContent = ""
            if (lead) {
                body.append(make("p", "hint", lead))
            }
            const { volumes } = await request("/api/storage/volumes")
            const list = make("div", "picker")
            for (const volume of volumes) {
                const item = make("button", "picker-item secondary")
                const free = volume.free_bytes === null
                    ? ""
                    : ` — ${bytesToText(volume.free_bytes)} free of ${bytesToText(volume.total_bytes)}`
                item.append(make("strong", "", volume.label || volume.path))
                item.append(make("span", "picker-detail", `${volume.path}${free}`))
                if (volume.removable) {
                    item.append(make("span", "pill", "removable"))
                }
                if (volume.read_only) {
                    item.disabled = true
                    item.append(make("span", "pill pill-bad", "read only"))
                }
                if (volume.free_bytes !== null && volume.free_bytes < needBytes) {
                    item.disabled = true
                    item.append(make("span", "pill pill-bad", "not enough room"))
                }
                item.addEventListener("click", () => showDirectory(volume.path))
                list.append(item)
            }
            if (volumes.length === 0) {
                list.append(make("p", "hint", "No writable volume is mounted. Try Auto-mount USB drives on the System tab."))
            }
            body.append(list)
        }

        const showDirectory = async (path) => {
            body.textContent = ""
            let listing = null
            try {
                listing = await request(`/api/storage/browse?path=${encodeURIComponent(path)}`)
            } catch (error) {
                body.append(make("p", "warning", error.message))
                const back = make("button", "secondary", "Back to drives")
                back.addEventListener("click", showVolumes)
                body.append(back)
                return
            }
            if (lead) {
                body.append(make("p", "hint", lead))
            }
            body.append(make("p", "path", listing.path))
            const free = listing.free_bytes === null ? "" : `${bytesToText(listing.free_bytes)} free`
            body.append(make("p", "hint", free))

            const list = make("div", "picker")
            const up = make("button", "picker-item secondary")
            up.append(make("strong", "", listing.parent ? ".." : "All drives"))
            up.addEventListener("click", () => (listing.parent ? showDirectory(listing.parent) : showVolumes()))
            list.append(up)
            for (const directory of listing.directories) {
                const item = make("button", "picker-item secondary")
                item.append(make("strong", "", directory.split("/").pop()))
                item.addEventListener("click", () => showDirectory(directory))
                list.append(item)
            }
            body.append(list)

            // A fresh drive has nowhere sensible to record into, and typing the
            // path was how that used to be solved.
            const add = make("button", "secondary", "New folder…")
            add.disabled = !listing.writable
            add.addEventListener("click", async () => {
                const name = prompt("Folder name")
                if (!name) {
                    return
                }
                try {
                    const made = await postJson("/api/storage/folder", { parent: listing.path, name })
                    showDirectory(made.path)
                } catch (error) {
                    toast(error.message, true)
                }
            })
            body.append(add)

            const choose = make("button", "", confirm)
            choose.disabled = !listing.writable
            if (!listing.writable) {
                body.append(make("p", "hint", "This folder cannot be written to."))
            }
            choose.addEventListener("click", async () => {
                choose.disabled = true
                try {
                    await onPick(listing.path)
                } catch (error) {
                    toast(error.message, true)
                    choose.disabled = false
                }
            })
            body.append(choose)
        }

        const opening = start ? showDirectory(start) : showVolumes()
        opening.catch((error) => {
            body.textContent = ""
            body.append(make("p", "warning", error.message))
        })
    })
}

const openTransferSheet = (file, kind) => openFolderPicker({
    title: `${kind === "copy" ? "Copy" : "Move"} ${file.name}`,
    lead: `${bytesToText(file.bytes)} to ${kind}. Pick a destination drive.`,
    confirm: kind === "copy" ? "Copy here" : "Move here",
    needBytes: file.bytes,
    onPick: async (destination) => {
        await postJson(`/api/recordings/${encodeURIComponent(file.name)}/${kind}`, { destination })
        closeSheet()
        // The copy outlives any request, so it is watched from the Files tab
        // where the operator can leave it running.
        showTab("files")
        watchMove()
    },
})

/** The `Save to` control on the settings tab: a path, and a way to change it. */
const renderRecordDir = () => {
    const button = element("record-dir")
    const path = state.settings?.record_dir || ""
    button.textContent = path || "choose a folder"
    button.title = path
}

const openRecordDirSheet = () => openFolderPicker({
    title: "Save recordings to",
    lead: "Recordings are written into the folder you choose.",
    confirm: "Save here",
    start: state.settings?.record_dir || null,
    onPick: async (path) => {
        state.settings.record_dir = path
        await saveSettings()
        // `saveSettings` swallows a refusal into a toast and reloads the real
        // settings, so the sheet closes on the value the server actually took.
        renderRecordDir()
        closeSheet()
    },
})

/** Polls a running move or copy until it ends, reporting into the Files tab. */
const watchMove = async () => {
    const box = element("transfer")
    while (true) {
        const status = await request("/api/move")
        box.hidden = !status.running
        if (status.running) {
            const done = status.copied_bytes + status.verified_bytes
            const parts = []
            if (status.total_work_bytes > 0) {
                parts.push(`${((done / status.total_work_bytes) * 100).toFixed(0)}%`)
            }
            if (status.bytes_per_second) {
                parts.push(`${bytesToText(status.bytes_per_second)}/s`)
            }
            if (status.eta_seconds !== null && status.eta_seconds !== undefined) {
                parts.push(`${secondsToText(status.eta_seconds)} left`)
            }
            // The read-back is the half that protects the original, so it is
            // named rather than left looking like a copy that has stalled.
            const verb = status.verifying
                ? "Checking"
                : status.kind === "copy" ? "Copying" : "Moving"
            const trail = parts.length > 0 ? ` — ${parts.join(" · ")}` : ""
            box.textContent = `${verb} ${status.source} to ${status.destination}${trail}`
            if (status.slow_link) {
                box.append(make("span", "warning-inline", " · slow link, USB 2.0 speed or worse"))
            }
            box.classList.toggle("notice-warn", Boolean(status.slow_link))
            await new Promise((resume) => setTimeout(resume, 700))
            continue
        }
        box.classList.remove("notice-warn")
        if (status.error) {
            toast(status.error, true)
        } else if (status.moved_to) {
            toast(`${status.kind === "copy" ? "copied" : "moved"} to ${status.moved_to}`)
        }
        refreshRecordings()
        return
    }
}

// -- recordings -----------------------------------------------------------

/** Conversions used to be written to `<name>.viewable.mcap`; such a file is already raw. */
const isConverted = (name) => name.endsWith(".viewable.mcap")

/**
 * Polls a running conversion until it ends. Watched here rather than from the
 * row, so the progress line survives the list being redrawn under it.
 */
const watchConversion = async () => {
    while (true) {
        const status = await request("/api/convert")
        const box = element("conversion")
        box.hidden = !status.running
        box.textContent = status.running
            ? `Post-processing ${status.source} — ${status.messages} messages, ${bytesToText(status.bytes)} written`
            : ""
        if (!status.running) {
            if (status.error) {
                toast(status.error, true)
            } else if (status.report) {
                const freed = status.report.reclaimed
                    ? `, ${bytesToText(status.report.reclaimed)} freed as it went`
                    : ""
                toast(`${status.output}: ${status.report.decoded} frames re-encoded${freed}`)
            }
            refreshRecordings()
            return
        }
        await new Promise((resume) => setTimeout(resume, 1000))
    }
}

const refreshRecordings = async () => {
    const files = await request("/api/recordings")
    const host = element("recording-list")
    host.textContent = ""
    if (files.length === 0) {
        host.append(make("p", "hint", "Nothing recorded yet."))
        return
    }
    for (const file of files) {
        const row = make("button", "file")
        row.type = "button"
        const head = make("div", "file-head")
        head.append(make("span", "file-name", file.name))
        head.append(make("span", "file-meta", `${timestampToText(file.modified)} · ${bytesToText(file.bytes)}`))
        row.append(head)
        row.append(make("span", "file-chevron", "›"))
        row.addEventListener("click", () => openFileSheet(file))
        host.append(row)
    }
}

/**
 * What you can do with one recording. A sheet rather than a row of buttons:
 * five of them never fitted on a phone, and laid out side by side they gave no
 * hint that one writes a new file and another destroys this one.
 */
const openFileSheet = (file) => {
    openSheet(file.name, (body) => {
        body.append(make("p", "hint", `${timestampToText(file.modified)} · ${bytesToText(file.bytes)}`))
        const actions = make("div", "actions")

        const action = (className, label, detail) => {
            const button = make("button", `action ${className}`)
            button.append(make("span", "", label))
            button.append(make("span", "action-detail", detail))
            actions.append(button)
            return button
        }

        action("secondary", "Preview", "Per-topic counts, rates and stalls, read from the file's index.")
            .addEventListener("click", () => openSummarySheet(file))

        // A real link, so the browser streams it to disk instead of buffering a
        // gigabyte inside the tab.
        const download = make("a", "button secondary action")
        download.href = `/api/recordings/${encodeURIComponent(file.name)}/download`
        download.setAttribute("download", file.name)
        download.append(make("span", "", "Download"))
        download.append(make("span", "action-detail", "Save the .mcap to this device."))
        actions.append(download)

        action("secondary", "Rename", "A new name in this folder. Instant; nothing is copied.")
            .addEventListener("click", async () => {
                // The marker a post-processed file carries is kept out of the
                // prompt and put back after, so renaming one does not make it
                // look unprocessed.
                const suffix = isConverted(file.name) ? ".viewable.mcap" : ".mcap"
                const current = file.name.endsWith(suffix) ? file.name.slice(0, -suffix.length) : file.name
                const wanted = prompt("New name", current)
                if (wanted === null || wanted.trim() === "" || wanted.trim() === current) {
                    return
                }
                try {
                    await postJson(`/api/recordings/${encodeURIComponent(file.name)}/rename`, { name: wanted.trim() + suffix })
                    closeSheet()
                    refreshRecordings()
                } catch (error) {
                    toast(error.message, true)
                }
            })

        if (!isConverted(file.name)) {
            const convert = action("secondary", "Post process", "Re-encode every image stream into something Foxglove and rerun can both draw: png colour and infrared, raw 16-bit depth. Lossless, and replaces this file in place — only if every frame decodes.")
            const start = (reclaim) =>
                postJson(`/api/recordings/${encodeURIComponent(file.name)}/convert${reclaim ? "?reclaim=true" : ""}`)
            convert.addEventListener("click", async () => {
                convert.disabled = true
                try {
                    await start(false)
                } catch (error) {
                    // The rewrite needs room for a second copy. When that is the
                    // only thing stopping it, the card can still do the job by
                    // giving each chunk back as it is converted — but that eats
                    // the original as it goes, so it takes a deliberate yes.
                    const tight = error.message.includes("not enough room")
                    if (!tight || !confirm(`${error.message}.\n\nConvert ${file.name} by freeing the original as it goes? Every chunk is checked before its space is released, but if this is interrupted the recording is left split across two files and has to be put back together by hand.`)) {
                        toast(error.message, true)
                        convert.disabled = false
                        return
                    }
                    try {
                        await start(true)
                    } catch (retry) {
                        toast(retry.message, true)
                        convert.disabled = false
                        return
                    }
                }
                closeSheet()
                watchConversion()
            })
        }

        action("secondary", "Copy", "Write onto another drive and read it back to check it, keeping this one.")
            .addEventListener("click", () => openTransferSheet(file, "copy"))

        action("secondary", "Move", "Write onto another drive, read it back to check it, then delete the original.")
            .addEventListener("click", () => openTransferSheet(file, "move"))

        const remove = action("secondary danger", "Delete", "Permanent. There is no trash on the recorder.")
        remove.addEventListener("click", async () => {
            // The size is in the prompt because it is what tells a stray tap on
            // a long recording apart from one on a ten second test.
            if (!confirm(`Delete ${file.name} (${bytesToText(file.bytes)})? This cannot be undone.`)) {
                return
            }
            try {
                await request(`/api/recordings/${encodeURIComponent(file.name)}`, { method: "DELETE" })
                closeSheet()
                refreshRecordings()
            } catch (error) {
                toast(error.message, true)
            }
        })

        body.append(actions)
    })
}

// -- tabs -----------------------------------------------------------------

const showTab = (name) => {
    for (const tab of document.querySelectorAll(".tab")) {
        tab.setAttribute("aria-selected", String(tab.dataset.tab === name))
    }
    for (const panel of document.querySelectorAll(".panel")) {
        panel.hidden = panel.dataset.panel !== name
    }
    // The charts are drawn into a hidden panel's SVG happily enough, but the
    // series only arrives once a second, so a freshly shown Monitor would be
    // blank for up to a second without this.
    if (name === "monitor") {
        renderCharts()
    }
    // Nothing else re-reads it: the status payload only arrives on a mutation,
    // so a password set from a second browser would leave this pill lying until
    // the page was reloaded.
    if (name === "system") {
        refreshAccess()
    }
    // In the address bar so a reload — which a rig on bad wifi gets a lot of —
    // comes back to the tab the operator was on rather than to the top.
    history.replaceState(null, "", `#${name}`)
    window.scrollTo({ top: 0 })
}

const knownTab = (name) => [...document.querySelectorAll(".tab")].some((tab) => tab.dataset.tab === name)

// -- wiring ---------------------------------------------------------------

const wire = () => {
    for (const tab of document.querySelectorAll(".tab")) {
        tab.addEventListener("click", () => showTab(tab.dataset.tab))
    }

    element("sheet-close").addEventListener("click", closeSheet)
    element("scrim").addEventListener("click", closeSheet)
    document.addEventListener("keydown", (event) => {
        if (event.key === "Escape" && !element("sheet").hidden) {
            closeSheet()
        }
    })

    element("record-toggle").addEventListener("click", async () => {
        const button = element("record-toggle")
        button.disabled = true
        try {
            if (state.recording.active) {
                renderRecording(await postJson("/api/record/stop"))
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

    element("record-dir").addEventListener("click", openRecordDirSheet)

    for (const control of document.querySelectorAll("[data-setting]")) {
        bindSettingControl(control)
    }

    element("access-change").addEventListener("click", openAccessSheet)
    element("access-forget").addEventListener("click", async () => {
        localStorage.removeItem(PASSWORD_KEY)
        await refreshAccess()
        toast("password forgotten on this browser")
    })

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
    showTab("system")
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
    const requested = location.hash.replace("#", "")
    if (knownTab(requested)) {
        showTab(requested)
    }
    try {
        await refreshStatus()
        await refreshAccess()
        await refreshRecordings()
        // A move started from another browser, or before this page was opened,
        // should still show its progress here.
        const transfer = await request("/api/move")
        if (transfer.running) {
            watchMove()
        }
    } catch (error) {
        toast(error.message, true)
    }
    startMonitorSocket()
    startPreviewSocket()
    startCloudView()
}

start()

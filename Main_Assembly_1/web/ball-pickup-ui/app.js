const sourceIds = ["F0", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8"];
const destinationIds = ["L0", "L1", "L2", "L3", "L4", "L5", "L6", "L7", "L8"];

const trayLayout = {
  F0: { x: 166, y: 186 }, F1: { x: 254, y: 186 }, F2: { x: 342, y: 186 },
  F3: { x: 166, y: 286 }, F4: { x: 254, y: 286 }, F5: { x: 342, y: 286 },
  F6: { x: 166, y: 386 }, F7: { x: 254, y: 386 }, F8: { x: 342, y: 386 },
  L0: { x: 662, y: 186 }, L1: { x: 750, y: 186 }, L2: { x: 838, y: 186 },
  L3: { x: 662, y: 286 }, L4: { x: 750, y: 286 }, L5: { x: 838, y: 286 },
  L6: { x: 662, y: 386 }, L7: { x: 750, y: 386 }, L8: { x: 838, y: 386 },
};

const state = {
  source: "F4",
  destination: "L0",
  running: false,
  heldBall: false,
  currentPoint: { x: 500, y: 336 },
};

const robot = {
  rootX: 500,
  rootY: 112,
  elbowSpan: 100,
  elbowHeight: 110,
};

const dom = {
  sourceGrid: document.getElementById("sourceGrid"),
  destinationGrid: document.getElementById("destinationGrid"),
  commandReadout: document.getElementById("commandReadout"),
  routeLabel: document.getElementById("routeLabel"),
  cycleState: document.getElementById("cycleState"),
  connectionState: document.getElementById("connectionState"),
  pressureLock: document.getElementById("pressureLock"),
  armPositionLabel: document.getElementById("armPositionLabel"),
  servoStateLabel: document.getElementById("servoStateLabel"),
  valveStateLabel: document.getElementById("valveStateLabel"),
  ballStateLabel: document.getElementById("ballStateLabel"),
  pressureRawLabel: document.getElementById("pressureRawLabel"),
  pressureValueLabel: document.getElementById("pressureValueLabel"),
  pressureFill: document.getElementById("pressureFill"),
  runCycleButton: document.getElementById("runCycleButton"),
  cancelCycleButton: document.getElementById("cancelCycleButton"),
  timelineList: document.getElementById("timelineList"),
  logFeed: document.getElementById("logFeed"),
  logTemplate: document.getElementById("logLineTemplate"),
  frontTrayCells: document.getElementById("frontTrayCells"),
  leftTrayCells: document.getElementById("leftTrayCells"),
  armLeft: document.getElementById("armLeft"),
  armRight: document.getElementById("armRight"),
  linkLeft: document.getElementById("linkLeft"),
  linkRight: document.getElementById("linkRight"),
  effector: document.getElementById("effector"),
  ballVisual: document.getElementById("ballVisual"),
};

let pollHandle = null;
let previousSnapshot = null;

function createButton(id, type) {
  const button = document.createElement("button");
  button.className = `hole-button hole-button--${type}`;
  button.type = "button";
  button.dataset.id = id;
  button.textContent = id;
  button.addEventListener("click", () => {
    if (state.running) return;
    state[type === "source" ? "source" : "destination"] = id;
    renderSelection();
    renderStageHighlights();
    addLog(`${type === "source" ? "Source" : "Destination"} updated to ${id}.`);
  });
  return button;
}

function buildGrid(container, ids, type) {
  ids.forEach((id) => container.appendChild(createButton(id, type)));
}

function makeSvgNode(name, attrs) {
  const node = document.createElementNS("http://www.w3.org/2000/svg", name);
  Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
  return node;
}

function drawStageCells() {
  const groups = {
    F: dom.frontTrayCells,
    L: dom.leftTrayCells,
  };

  Object.entries(trayLayout).forEach(([id, position]) => {
    const group = makeSvgNode("g", { "data-hole": id });
    const rect = makeSvgNode("rect", {
      x: position.x - 30,
      y: position.y - 30,
      width: 60,
      height: 60,
      rx: 18,
      class: "tray-cell",
    });
    const text = makeSvgNode("text", {
      x: position.x,
      y: position.y + 5,
      class: "tray-label",
    });
    text.textContent = id;
    group.append(rect, text);
    groups[id[0]].appendChild(group);
  });
}

function renderSelection() {
  dom.commandReadout.textContent = `${state.source} → ${state.destination}`;
  dom.routeLabel.textContent = `${state.source} → ${state.destination}`;

  document.querySelectorAll(".hole-button--source").forEach((button) => {
    button.classList.toggle("is-selected", button.dataset.id === state.source);
  });

  document.querySelectorAll(".hole-button--destination").forEach((button) => {
    button.classList.toggle("is-selected", button.dataset.id === state.destination);
  });
}

function renderStageHighlights() {
  document.querySelectorAll("[data-hole]").forEach((group) => {
    const cell = group.querySelector(".tray-cell");
    cell.classList.remove("tray-cell--source", "tray-cell--destination", "tray-cell--highlight");
    if (group.dataset.hole === state.source) {
      cell.classList.add("tray-cell--source", "tray-cell--highlight");
    }
    if (group.dataset.hole === state.destination) {
      cell.classList.add("tray-cell--destination", "tray-cell--highlight");
    }
  });
}

function setPressure(level, label, lockText) {
  dom.pressureFill.style.width = `${Math.max(0, Math.min(level, 100))}%`;
  dom.pressureValueLabel.textContent = label;
  dom.pressureLock.textContent = lockText;
}

function setTimeline(activeIndex, doneUntil = activeIndex - 1) {
  [...dom.timelineList.children].forEach((item, index) => {
    item.classList.remove("timeline__item--active", "timeline__item--done");
    if (index < doneUntil + 1) item.classList.add("timeline__item--done");
    if (index === activeIndex) item.classList.add("timeline__item--active");
  });
}

function addLog(text) {
  const fragment = dom.logTemplate.content.cloneNode(true);
  const now = new Date();
  fragment.querySelector(".log__time").textContent = now.toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
  fragment.querySelector(".log__text").textContent = text;
  dom.logFeed.prepend(fragment);
  while (dom.logFeed.children.length > 10) {
    dom.logFeed.removeChild(dom.logFeed.lastChild);
  }
}

function updateArm(point) {
  state.currentPoint = point;
  const leftElbow = { x: -robot.elbowSpan, y: robot.elbowHeight };
  const rightElbow = { x: robot.elbowSpan, y: robot.elbowHeight };
  const effector = { x: point.x - robot.rootX, y: point.y - robot.rootY };

  dom.armLeft.setAttribute("x2", leftElbow.x);
  dom.armLeft.setAttribute("y2", leftElbow.y);
  dom.armRight.setAttribute("x2", rightElbow.x);
  dom.armRight.setAttribute("y2", rightElbow.y);
  dom.linkLeft.setAttribute("x1", leftElbow.x);
  dom.linkLeft.setAttribute("y1", leftElbow.y);
  dom.linkLeft.setAttribute("x2", effector.x);
  dom.linkLeft.setAttribute("y2", effector.y);
  dom.linkRight.setAttribute("x1", rightElbow.x);
  dom.linkRight.setAttribute("y1", rightElbow.y);
  dom.linkRight.setAttribute("x2", effector.x);
  dom.linkRight.setAttribute("y2", effector.y);
  dom.effector.setAttribute("transform", `translate(${effector.x} ${effector.y})`);
}

function interpolate(a, b, t) {
  return a + (b - a) * t;
}

function easeInOut(t) {
  return t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
}

function animateMove(target, duration = 900) {
  const start = { ...state.currentPoint };
  return new Promise((resolve) => {
    const startTime = performance.now();

    function frame(now) {
      const elapsed = now - startTime;
      const t = Math.min(1, elapsed / duration);
      const eased = easeInOut(t);
      updateArm({
        x: interpolate(start.x, target.x, eased),
        y: interpolate(start.y, target.y, eased),
      });
      if (t < 1) {
        requestAnimationFrame(frame);
      } else {
        resolve();
      }
    }

    requestAnimationFrame(frame);
  });
}

function pause(duration) {
  return new Promise((resolve) => window.setTimeout(resolve, duration));
}

async function sendCommand(payload) {
  const response = await fetch("/api/command", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(payload),
  });

  if (!response.ok) {
    throw new Error(`Command failed: ${response.status}`);
  }

  return response.json();
}

function bootLogs() {
  [
    "Digital twin initialized.",
    "Pressure threshold profile loaded.",
    "Kinematic envelope verified.",
    "Standby path clear. Awaiting route selection.",
  ].forEach(addLog);
}

function plannerStateName(snapshot) {
  return snapshot.planner_state || "IDLE";
}

function plannerTimelineIndex(plannerState) {
  switch (plannerState) {
    case "MOVE_ABOVE_SRC":
      return { active: 0, done: -1 };
    case "DESCEND_SRC":
      return { active: 0, done: 0 };
    case "SUCTION_ON":
      return { active: 2, done: 1 };
    case "ASCEND_SRC":
    case "RETRACT_SRC":
    case "TRANSFER":
    case "MOVE_ABOVE_DST":
      return { active: 3, done: 2 };
    case "DESCEND_DST":
    case "SUCTION_OFF":
    case "ASCEND_DST":
    case "RETRACT_DST":
      return { active: 4, done: 3 };
    case "MOVE_HOME":
      return { active: 4, done: 4 };
    default:
      return { active: 0, done: -1 };
  }
}

function ikToStagePoint(ikTarget) {
  const x = 500 + ikTarget.x * 3600;
  const y = 780 - ikTarget.y * 1300;
  return {
    x: Math.max(130, Math.min(870, x)),
    y: Math.max(170, Math.min(470, y)),
  };
}

function servoLabel(plannerState, suctionOn) {
  if (plannerState === "SUCTION_ON" || plannerState === "SUCTION_OFF") {
    return "Down / active";
  }
  if (plannerState === "DESCEND_SRC" || plannerState === "DESCEND_DST") {
    return "Lowering";
  }
  if (suctionOn) {
    return "Raised / holding";
  }
  return "Raised";
}

function pressureUi(snapshot) {
  const raw = snapshot.pressure_raw;
  if (raw == null) {
    return { fill: 8, label: "No sensor data", lock: "Waiting", rawLabel: "--" };
  }
  const fill = Math.round((raw / 16777215) * 100);
  const label = `raw_u24 ${raw}`;
  const lock = snapshot.ball_detected ? "Pick confirmed" : "Monitoring";
  return { fill, label, lock, rawLabel: String(raw) };
}

function updateFromSnapshot(snapshot) {
  dom.connectionState.textContent = snapshot.connected ? "ROS linked" : "Offline";

  if (snapshot.route?.source) {
    state.source = snapshot.route.source;
  }
  if (snapshot.route?.destination) {
    state.destination = snapshot.route.destination;
  }

  renderSelection();
  renderStageHighlights();

  const plannerState = plannerStateName(snapshot);
  dom.cycleState.textContent = plannerState.replaceAll("_", " ");
  dom.armPositionLabel.textContent = snapshot.ik_target
    ? `IK (${snapshot.ik_target.x.toFixed(3)}, ${snapshot.ik_target.y.toFixed(3)})`
    : "Home Arc";
  dom.servoStateLabel.textContent = servoLabel(plannerState, snapshot.suction_on);
  dom.valveStateLabel.textContent = snapshot.suction_on ? "Valve Closed" : "Vent Open";
  dom.ballStateLabel.textContent = snapshot.ball_status || (snapshot.ball_detected ? "Ball detected" : "No pick confirmed");

  const pressure = pressureUi(snapshot);
  setPressure(pressure.fill, pressure.label, pressure.lock);
  dom.pressureRawLabel.textContent = pressure.rawLabel;

  const timeline = plannerTimelineIndex(plannerState);
  setTimeline(timeline.active, timeline.done);

  if (snapshot.ik_target) {
    updateArm(ikToStagePoint(snapshot.ik_target));
  }

  dom.ballVisual.classList.toggle("hidden", !snapshot.suction_on && !snapshot.ball_detected);
}

function diffLogs(snapshot) {
  if (!previousSnapshot) {
    addLog("Live dashboard connected to ROS bridge.");
    return;
  }

  if (snapshot.planner_status !== previousSnapshot.planner_status) {
    addLog(`Planner: ${snapshot.planner_status}`);
  }
  if (snapshot.arduino_status !== previousSnapshot.arduino_status && snapshot.arduino_status) {
    addLog(`Arduino: ${snapshot.arduino_status}`);
  }
  if (snapshot.ball_status !== previousSnapshot.ball_status && snapshot.ball_status) {
    addLog(`Ball: ${snapshot.ball_status}`);
  }
  if (snapshot.ball_detected !== previousSnapshot.ball_detected) {
    addLog(snapshot.ball_detected ? "Pressure sensor confirmed pickup." : "Pressure confirmation cleared.");
  }
}

async function pollState() {
  try {
    const response = await fetch("/api/state", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`State poll failed: ${response.status}`);
    }
    const snapshot = await response.json();
    snapshot.connected = true;
    diffLogs(snapshot);
    updateFromSnapshot(snapshot);
    previousSnapshot = snapshot;
    dom.runCycleButton.disabled = false;
    dom.cancelCycleButton.disabled = false;
  } catch (error) {
    dom.connectionState.textContent = "Disconnected";
    dom.pressureLock.textContent = "Bridge unavailable";
    dom.runCycleButton.disabled = false;
    dom.cancelCycleButton.disabled = false;
  }
}

function init() {
  buildGrid(dom.sourceGrid, sourceIds, "source");
  buildGrid(dom.destinationGrid, destinationIds, "destination");
  drawStageCells();
  renderSelection();
  renderStageHighlights();
  updateArm({ x: 500, y: 336 });
  bootLogs();
  dom.runCycleButton.addEventListener("click", async () => {
    dom.runCycleButton.disabled = true;
    try {
      await sendCommand({ action: "pick_place", source: state.source, destination: state.destination });
      addLog(`Command sent: ${state.source} → ${state.destination}`);
    } catch (error) {
      addLog(`Command error: ${error.message}`);
    } finally {
      dom.runCycleButton.disabled = false;
    }
  });
  dom.cancelCycleButton.addEventListener("click", async () => {
    dom.cancelCycleButton.disabled = true;
    try {
      await sendCommand({ action: "cancel" });
      addLog("Cancel command sent.");
    } catch (error) {
      addLog(`Cancel error: ${error.message}`);
    } finally {
      dom.cancelCycleButton.disabled = false;
    }
  });
  pollState();
  pollHandle = window.setInterval(pollState, 350);
}

init();
const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const statsEl = document.getElementById("stats");
const toggleBtn = document.getElementById("toggleBtn");
const resetBtn = document.getElementById("resetBtn");
const newMapBtn = document.getElementById("newMapBtn");
const learnBtn = document.getElementById("learnBtn");
const resetBrainBtn = document.getElementById("resetBrainBtn");
const robotSelect = document.getElementById("robotSelect");
const robotLegendEl = document.getElementById("robotLegend");

const WORLD = {
  width: canvas.width,
  height: canvas.height,
  wallPadding: 10,
};

const SIM = {
  running: true,
  dtMax: 0.03,
  time: 0,
};

const ROBOT_BASE = {
  radius: 16,
  wheelBase: 34,
  maxWheelSpeed: 120,
};

const ROBOT_COUNT = 10;

const BRAIN_CFG = {
  inputSize: 9,
  hiddenSize: 14,
  outputSize: 2,
};

const TRAIN_DEFAULT = {
  enabled: true,
  lr: 0.003,
  sigma: 0.36,
  sigmaMin: 0.06,
  sigmaDecay: 0.99995,
};

let obstacles = [];
let light = null;
let robots = [];
let selectedRobotIndex = 0;
let globalLearningEnabled = true;

function rand(min, max) {
  return Math.random() * (max - min) + min;
}

function randn() {
  let u = 0;
  let v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function circleRectCollision(cx, cy, r, rect) {
  const closestX = clamp(cx, rect.x, rect.x + rect.w);
  const closestY = clamp(cy, rect.y, rect.y + rect.h);
  const dx = cx - closestX;
  const dy = cy - closestY;
  return dx * dx + dy * dy <= r * r;
}

function lineRectDistance(x, y, angle, maxDist, rect) {
  const step = 4;
  for (let d = 0; d <= maxDist; d += step) {
    const sx = x + Math.cos(angle) * d;
    const sy = y + Math.sin(angle) * d;
    if (sx < 0 || sy < 0 || sx > WORLD.width || sy > WORLD.height) return d;
    if (sx >= rect.x && sx <= rect.x + rect.w && sy >= rect.y && sy <= rect.y + rect.h) return d;
  }
  return maxDist;
}

function distanceToObstacle(x, y, angle, maxDist) {
  let nearest = maxDist;
  for (const rect of obstacles) {
    const d = lineRectDistance(x, y, angle, maxDist, rect);
    if (d < nearest) nearest = d;
  }
  const wallD = [
    angleCosDistance(x, Math.cos(angle), 0, WORLD.width),
    angleCosDistance(y, Math.sin(angle), 0, WORLD.height),
  ];
  nearest = Math.min(nearest, wallD[0], wallD[1]);
  return nearest;
}

function angleCosDistance(p, dir, min, max) {
  if (Math.abs(dir) < 1e-6) return Infinity;
  if (dir > 0) return (max - p) / dir;
  return (min - p) / dir;
}

function lightIntensity(x, y) {
  const dx = x - light.x;
  const dy = y - light.y;
  const d2 = dx * dx + dy * dy;
  return Math.exp(-d2 / (2 * light.sigma * light.sigma));
}

function blocked(x, y, r) {
  if (x < WORLD.wallPadding + r || y < WORLD.wallPadding + r) return true;
  if (x > WORLD.width - WORLD.wallPadding - r || y > WORLD.height - WORLD.wallPadding - r) return true;
  return obstacles.some((rect) => circleRectCollision(x, y, r, rect));
}

function generateMap() {
  obstacles = [];
  const obstacleCount = 12;
  const margin = 80;
  for (let i = 0; i < obstacleCount; i++) {
    const w = rand(45, 120);
    const h = rand(35, 100);
    obstacles.push({
      x: rand(margin, WORLD.width - margin - w),
      y: rand(margin, WORLD.height - margin - h),
      w,
      h,
    });
  }

  let lx = 0;
  let ly = 0;
  for (let i = 0; i < 200; i++) {
    lx = rand(80, WORLD.width - 80);
    ly = rand(80, WORLD.height - 80);
    if (!blocked(lx, ly, 30)) break;
  }
  light = { x: lx, y: ly, sigma: 110 };
}

function createBrain(inputSize, hiddenSize, outputSize) {
  const w1 = Array.from({ length: hiddenSize }, () =>
    Array.from({ length: inputSize }, () => rand(-0.25, 0.25))
  );
  const b1 = Array.from({ length: hiddenSize }, () => 0);
  const w2 = Array.from({ length: outputSize }, () =>
    Array.from({ length: hiddenSize }, () => rand(-0.25, 0.25))
  );
  const b2 = Array.from({ length: outputSize }, () => 0);
  return { inputSize, hiddenSize, outputSize, w1, b1, w2, b2 };
}

function createTrainState(enabled) {
  return {
    enabled,
    lr: TRAIN_DEFAULT.lr,
    sigma: TRAIN_DEFAULT.sigma,
    sigmaMin: TRAIN_DEFAULT.sigmaMin,
    sigmaDecay: TRAIN_DEFAULT.sigmaDecay,
    rewardBaseline: 0,
    rewardAvg: 0,
    lastReward: 0,
    steps: 0,
    collisions: 0,
    goals: 0,
  };
}

function sampleValidPose() {
  let x = 0;
  let y = 0;
  for (let i = 0; i < 300; i++) {
    x = rand(50, WORLD.width - 50);
    y = rand(50, WORLD.height - 50);
    if (!blocked(x, y, ROBOT_BASE.radius + 4)) break;
  }
  return { x, y, theta: rand(0, Math.PI * 2) };
}

function robotColor(index) {
  const hue = (index * 36) % 360;
  return `hsl(${hue} 68% 58%)`;
}

function createRobot(index) {
  const pose = sampleValidPose();
  const initialIntensity = lightIntensity(pose.x, pose.y);
  return {
    index,
    color: robotColor(index),
    x: pose.x,
    y: pose.y,
    theta: pose.theta,
    vl: 0,
    vr: 0,
    trail: [],
    bestIntensity: initialIntensity,
    prevIntensity: initialIntensity,
    lastSensors: null,
    lastAction: [0, 0],
    lastPolicyMean: [0, 0],
    brain: createBrain(BRAIN_CFG.inputSize, BRAIN_CFG.hiddenSize, BRAIN_CFG.outputSize),
    train: createTrainState(globalLearningEnabled),
  };
}

function resetRobotPose(robot) {
  const pose = sampleValidPose();
  const initialIntensity = lightIntensity(pose.x, pose.y);
  robot.x = pose.x;
  robot.y = pose.y;
  robot.theta = pose.theta;
  robot.vl = 0;
  robot.vr = 0;
  robot.trail = [];
  robot.bestIntensity = initialIntensity;
  robot.prevIntensity = initialIntensity;
  robot.lastSensors = null;
  robot.lastAction = [0, 0];
  robot.lastPolicyMean = [0, 0];
}

function resetRobotBrain(robot) {
  robot.brain = createBrain(BRAIN_CFG.inputSize, BRAIN_CFG.hiddenSize, BRAIN_CFG.outputSize);
  robot.train = createTrainState(globalLearningEnabled);
}

function spawnRobots(count) {
  robots = [];
  for (let i = 0; i < count; i++) {
    robots.push(createRobot(i));
  }
  selectedRobotIndex = clamp(selectedRobotIndex, 0, robots.length - 1);
  populateRobotSelector();
}

function populateRobotSelector() {
  if (!robotSelect) return;
  robotSelect.innerHTML = "";
  for (let i = 0; i < robots.length; i++) {
    const option = document.createElement("option");
    option.value = String(i);
    option.textContent = `Robot ${i + 1}`;
    robotSelect.appendChild(option);
  }
  robotSelect.value = String(selectedRobotIndex);
  renderRobotLegend();
}

function renderRobotLegend() {
  if (!robotLegendEl) return;
  robotLegendEl.innerHTML = "";
  for (let i = 0; i < robots.length; i++) {
    const robot = robots[i];
    const item = document.createElement("button");
    item.type = "button";
    item.className = `legend-item ${i === selectedRobotIndex ? "selected" : ""}`;
    item.innerHTML = `<span class="legend-swatch" style="background:${robot.color}"></span><span>R${i + 1}</span>`;
    item.addEventListener("click", () => {
      selectedRobotIndex = i;
      if (robotSelect) robotSelect.value = String(i);
      renderRobotLegend();
    });
    robotLegendEl.appendChild(item);
  }
}

function sense(robotState) {
  const sensorAngles = [-0.6, 0, 0.6];
  const range = 130;
  const sensors = sensorAngles.map((a) => {
    const heading = robotState.theta + a;
    const d = distanceToObstacle(robotState.x, robotState.y, heading, range);
    const sx = robotState.x + Math.cos(heading) * 18;
    const sy = robotState.y + Math.sin(heading) * 18;
    const b = lightIntensity(sx + Math.cos(heading) * 18, sy + Math.sin(heading) * 18);
    return {
      angle: heading,
      distance: d,
      brightness: b,
      maxRange: range,
    };
  });
  return sensors;
}

function featuresFromSensors(sensors) {
  const [left, center, right] = sensors;
  const leftDist = clamp(left.distance / left.maxRange, 0, 1);
  const centerDist = clamp(center.distance / center.maxRange, 0, 1);
  const rightDist = clamp(right.distance / right.maxRange, 0, 1);
  const brightDiff = right.brightness - left.brightness;
  const centerAvoid = 1 - centerDist;
  return [
    leftDist,
    centerDist,
    rightDist,
    left.brightness,
    center.brightness,
    right.brightness,
    brightDiff,
    centerAvoid,
    1,
  ];
}

function forwardBrain(model, input) {
  const hidden = new Array(model.hiddenSize);
  for (let i = 0; i < model.hiddenSize; i++) {
    let sum = model.b1[i];
    for (let j = 0; j < model.inputSize; j++) {
      sum += model.w1[i][j] * input[j];
    }
    hidden[i] = Math.tanh(sum);
  }

  const mean = new Array(model.outputSize);
  for (let i = 0; i < model.outputSize; i++) {
    let sum = model.b2[i];
    for (let j = 0; j < model.hiddenSize; j++) {
      sum += model.w2[i][j] * hidden[j];
    }
    mean[i] = Math.tanh(sum);
  }

  return { input, hidden, mean };
}

function sampleAction(mean, sigma) {
  return mean.map((m) => clamp(m + randn() * sigma, -1, 1));
}

function applyPolicyGradient(model, cache, action, sigma, advantage, lr) {
  const dMu = new Array(model.outputSize);
  const sigma2 = sigma * sigma;
  for (let i = 0; i < model.outputSize; i++) {
    dMu[i] = (advantage * (action[i] - cache.mean[i])) / sigma2;
  }

  const dOut = new Array(model.outputSize);
  for (let i = 0; i < model.outputSize; i++) {
    dOut[i] = dMu[i] * (1 - cache.mean[i] * cache.mean[i]);
  }

  const dHidden = new Array(model.hiddenSize).fill(0);
  for (let i = 0; i < model.outputSize; i++) {
    for (let j = 0; j < model.hiddenSize; j++) {
      dHidden[j] += model.w2[i][j] * dOut[i];
      model.w2[i][j] += lr * dOut[i] * cache.hidden[j];
      model.w2[i][j] = clamp(model.w2[i][j], -3, 3);
    }
    model.b2[i] += lr * dOut[i];
    model.b2[i] = clamp(model.b2[i], -3, 3);
  }

  for (let i = 0; i < model.hiddenSize; i++) {
    const dPre = dHidden[i] * (1 - cache.hidden[i] * cache.hidden[i]);
    for (let j = 0; j < model.inputSize; j++) {
      model.w1[i][j] += lr * dPre * cache.input[j];
      model.w1[i][j] = clamp(model.w1[i][j], -3, 3);
    }
    model.b1[i] += lr * dPre;
    model.b1[i] = clamp(model.b1[i], -3, 3);
  }
}

function applyAction(robotState, action, dt) {
  const forward = action[0] * 95;
  const turn = action[1] * 3.2;

  robotState.vl = clamp(
    forward - (turn * ROBOT_BASE.wheelBase) / 2,
    -ROBOT_BASE.maxWheelSpeed,
    ROBOT_BASE.maxWheelSpeed
  );
  robotState.vr = clamp(
    forward + (turn * ROBOT_BASE.wheelBase) / 2,
    -ROBOT_BASE.maxWheelSpeed,
    ROBOT_BASE.maxWheelSpeed
  );

  const linear = (robotState.vl + robotState.vr) / 2;
  const angular = (robotState.vr - robotState.vl) / ROBOT_BASE.wheelBase;
  const nx = robotState.x + Math.cos(robotState.theta) * linear * dt;
  const ny = robotState.y + Math.sin(robotState.theta) * linear * dt;
  const nt = robotState.theta + angular * dt;

  let collision = false;
  if (!blocked(nx, ny, ROBOT_BASE.radius)) {
    robotState.x = nx;
    robotState.y = ny;
    robotState.theta = nt;
  } else {
    collision = true;
    robotState.theta += (Math.random() - 0.5) * 0.8;
    robotState.vl *= 0.3;
    robotState.vr *= 0.3;
  }

  return collision;
}

function stepNeuralController(robotState, sensors, dt) {
  const cache = forwardBrain(robotState.brain, featuresFromSensors(sensors));
  const action = sampleAction(cache.mean, robotState.train.sigma);
  const collision = applyAction(robotState, action, dt);

  const intensity = lightIntensity(robotState.x, robotState.y);
  const dToGoal = Math.hypot(robotState.x - light.x, robotState.y - light.y);
  const deltaBrightness = intensity - robotState.prevIntensity;
  const nearObstacle = 1 - clamp(sensors[1].distance / sensors[1].maxRange, 0, 1);

  let reward = 5.4 * deltaBrightness;
  reward += 0.025 * intensity;
  reward -= 0.008;
  reward -= 0.03 * nearObstacle;
  if (collision) reward -= 0.08;
  if (dToGoal < ROBOT_BASE.radius + 15) {
    reward += 0.12;
    robotState.train.goals += 1;
  }

  robotState.train.lastReward = reward;
  robotState.train.rewardBaseline = 0.995 * robotState.train.rewardBaseline + 0.005 * reward;
  robotState.train.rewardAvg = 0.995 * robotState.train.rewardAvg + 0.005 * reward;

  if (robotState.train.enabled) {
    const advantage = reward - robotState.train.rewardBaseline;
    applyPolicyGradient(robotState.brain, cache, action, robotState.train.sigma, advantage, robotState.train.lr);
    robotState.train.sigma = Math.max(robotState.train.sigmaMin, robotState.train.sigma * robotState.train.sigmaDecay);
  }

  robotState.train.steps += 1;
  if (collision) robotState.train.collisions += 1;

  robotState.prevIntensity = intensity;
  robotState.bestIntensity = Math.max(robotState.bestIntensity, intensity);
  robotState.trail.push({ x: robotState.x, y: robotState.y });
  if (robotState.trail.length > 900) robotState.trail.shift();

  robotState.lastSensors = sensors;
  robotState.lastAction = action;
  robotState.lastPolicyMean = cache.mean;
}

function drawLightField() {
  const step = 14;
  for (let y = 0; y < WORLD.height; y += step) {
    for (let x = 0; x < WORLD.width; x += step) {
      const b = lightIntensity(x, y);
      if (b < 0.015) continue;
      const alpha = clamp(b * 0.28, 0, 0.32);
      ctx.fillStyle = `rgba(255, 215, 90, ${alpha})`;
      ctx.fillRect(x, y, step, step);
    }
  }
}

function drawObstacles() {
  ctx.fillStyle = "#334a71";
  ctx.strokeStyle = "#8fb4f9";
  for (const rect of obstacles) {
    ctx.fillRect(rect.x, rect.y, rect.w, rect.h);
    ctx.strokeRect(rect.x, rect.y, rect.w, rect.h);
  }
}

function drawTrail(robotState, selected) {
  if (robotState.trail.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(robotState.trail[0].x, robotState.trail[0].y);
  for (let i = 1; i < robotState.trail.length; i++) {
    ctx.lineTo(robotState.trail[i].x, robotState.trail[i].y);
  }
  ctx.strokeStyle = selected ? "rgba(255, 140, 66, 0.62)" : "rgba(155, 190, 255, 0.22)";
  ctx.lineWidth = selected ? 2.2 : 1.3;
  ctx.stroke();
}

function drawRobot(robotState, selected) {
  const sensors = robotState.lastSensors || sense(robotState);
  for (const s of sensors) {
    const ex = robotState.x + Math.cos(s.angle) * s.distance;
    const ey = robotState.y + Math.sin(s.angle) * s.distance;
    ctx.beginPath();
    ctx.moveTo(robotState.x, robotState.y);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = selected ? "rgba(190, 220, 255, 0.45)" : "rgba(190, 220, 255, 0.12)";
    ctx.stroke();
  }

  ctx.beginPath();
  ctx.arc(robotState.x, robotState.y, ROBOT_BASE.radius, 0, Math.PI * 2);
  ctx.fillStyle = robotState.color;
  ctx.fill();
  ctx.strokeStyle = selected ? "#ffd6b7" : "rgba(245, 250, 255, 0.45)";
  ctx.lineWidth = selected ? 2.1 : 1.2;
  ctx.stroke();

  const hx = robotState.x + Math.cos(robotState.theta) * ROBOT_BASE.radius;
  const hy = robotState.y + Math.sin(robotState.theta) * ROBOT_BASE.radius;
  ctx.beginPath();
  ctx.moveTo(robotState.x, robotState.y);
  ctx.lineTo(hx, hy);
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = selected ? 2.2 : 1.2;
  ctx.stroke();
}

function drawGoal() {
  const pulse = 6 + 3 * Math.sin(SIM.time * 2.6);
  ctx.beginPath();
  ctx.arc(light.x, light.y, pulse + 8, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(255, 229, 145, 0.18)";
  ctx.fill();
  ctx.beginPath();
  ctx.arc(light.x, light.y, 6, 0, Math.PI * 2);
  ctx.fillStyle = "#ffe68a";
  ctx.fill();
  ctx.strokeStyle = "#fff7ca";
  ctx.stroke();
}

function drawFrame() {
  ctx.clearRect(0, 0, WORLD.width, WORLD.height);
  drawLightField();
  drawObstacles();
  for (let i = 0; i < robots.length; i++) {
    drawTrail(robots[i], i === selectedRobotIndex);
  }
  drawGoal();
  for (let i = 0; i < robots.length; i++) {
    drawRobot(robots[i], i === selectedRobotIndex);
  }
}

function updateStats() {
  const robot = robots[selectedRobotIndex];
  if (!robot) return;

  const intensity = lightIntensity(robot.x, robot.y);
  const dToGoal = Math.hypot(robot.x - light.x, robot.y - light.y);
  const avgBest = robots.reduce((acc, r) => acc + r.bestIntensity, 0) / robots.length;

  statsEl.innerHTML = [
    `Viewing: Robot ${robot.index + 1} / ${robots.length}`,
    `Time: ${SIM.time.toFixed(1)} s`,
    `Position: (${robot.x.toFixed(1)}, ${robot.y.toFixed(1)})`,
    `Heading: ${(robot.theta % (Math.PI * 2)).toFixed(2)} rad`,
    `Wheel Speeds (L/R): ${robot.vl.toFixed(1)} / ${robot.vr.toFixed(1)}`,
    `Current Brightness: ${intensity.toFixed(3)}`,
    `Best Brightness (this robot): ${robot.bestIntensity.toFixed(3)}`,
    `Best Brightness (fleet avg): ${avgBest.toFixed(3)}`,
    `Distance to Brightest Spot: ${dToGoal.toFixed(1)} px`,
    `Learning: ${robot.train.enabled ? "On" : "Off"}`,
    `Training Steps: ${robot.train.steps}`,
    `Exploration Sigma: ${robot.train.sigma.toFixed(3)}`,
    `Learning Rate: ${robot.train.lr.toFixed(4)}`,
    `Reward (avg / last): ${robot.train.rewardAvg.toFixed(4)} / ${robot.train.lastReward.toFixed(4)}`,
    `Policy Output (v,w): ${robot.lastAction[0].toFixed(2)} / ${robot.lastAction[1].toFixed(2)}`,
    `Collisions: ${robot.train.collisions} | Goal Hits: ${robot.train.goals}`,
  ].join("<br>");
}

let lastTs = 0;
function loop(ts) {
  if (!lastTs) lastTs = ts;
  const dt = Math.min((ts - lastTs) / 1000, SIM.dtMax);
  lastTs = ts;

  if (SIM.running) {
    for (const robot of robots) {
      const sensors = sense(robot);
      stepNeuralController(robot, sensors, dt);
    }
    SIM.time += dt;
  }

  drawFrame();
  updateStats();
  requestAnimationFrame(loop);
}

toggleBtn.addEventListener("click", () => {
  SIM.running = !SIM.running;
  toggleBtn.textContent = SIM.running ? "Pause" : "Resume";
});

resetBtn.addEventListener("click", () => {
  const robot = robots[selectedRobotIndex];
  if (!robot) return;
  resetRobotPose(robot);
});

newMapBtn.addEventListener("click", () => {
  generateMap();
  for (const robot of robots) {
    resetRobotPose(robot);
  }
  SIM.time = 0;
});

if (robotSelect) {
  robotSelect.addEventListener("change", () => {
    selectedRobotIndex = clamp(Number(robotSelect.value) || 0, 0, robots.length - 1);
    renderRobotLegend();
  });
}

if (learnBtn) {
  learnBtn.addEventListener("click", () => {
    globalLearningEnabled = !globalLearningEnabled;
    for (const robot of robots) {
      robot.train.enabled = globalLearningEnabled;
    }
    learnBtn.textContent = `Learning: ${globalLearningEnabled ? "On" : "Off"}`;
  });
}

if (resetBrainBtn) {
  resetBrainBtn.addEventListener("click", () => {
    const robot = robots[selectedRobotIndex];
    if (!robot) return;
    resetRobotBrain(robot);
    resetRobotPose(robot);
  });
}

generateMap();
spawnRobots(ROBOT_COUNT);
if (learnBtn) {
  learnBtn.textContent = `Learning: ${globalLearningEnabled ? "On" : "Off"}`;
}
requestAnimationFrame(loop);

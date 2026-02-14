const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");
const statsEl = document.getElementById("stats");
const toggleBtn = document.getElementById("toggleBtn");
const resetBtn = document.getElementById("resetBtn");
const newMapBtn = document.getElementById("newMapBtn");

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

let obstacles = [];
let light = null;
let robot = null;

function rand(min, max) {
  return Math.random() * (max - min) + min;
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

function resetRobot() {
  let x = 0;
  let y = 0;
  for (let i = 0; i < 300; i++) {
    x = rand(50, WORLD.width - 50);
    y = rand(50, WORLD.height - 50);
    if (!blocked(x, y, ROBOT_BASE.radius + 4)) break;
  }

  robot = {
    x,
    y,
    theta: rand(0, Math.PI * 2),
    vl: 0,
    vr: 0,
    trail: [],
    bestIntensity: 0,
  };
}

function sense(robotState) {
  const sensorAngles = [-0.6, 0, 0.6];
  const range = 130;
  const sensors = sensorAngles.map((a) => {
    const heading = robotState.theta + a;
    const d = distanceToObstacle(robotState.x, robotState.y, heading, range);
    const sx = robotState.x + Math.cos(heading) * 18;
    const sy = robotState.y + Math.sin(heading) * 18;
    const b = lightIntensity(
      sx + Math.cos(heading) * 18,
      sy + Math.sin(heading) * 18
    );
    return {
      angle: heading,
      distance: d,
      brightness: b,
      maxRange: range,
    };
  });
  return sensors;
}

function controller(robotState, sensors, dt) {
  const [left, center, right] = sensors;
  const avoidL = 1 - clamp(left.distance / left.maxRange, 0, 1);
  const avoidC = 1 - clamp(center.distance / center.maxRange, 0, 1);
  const avoidR = 1 - clamp(right.distance / right.maxRange, 0, 1);

  const brightDiff = right.brightness - left.brightness;
  const centeredBright = center.brightness;

  let forward = 45 + 80 * centeredBright;
  let turn = 120 * brightDiff;

  // Strong repulsion when obstacles are close in front.
  forward -= 95 * avoidC;
  turn += 220 * (avoidL - avoidR);

  // Escape behavior when boxed in.
  if (avoidC > 0.85 && (avoidL > 0.65 || avoidR > 0.65)) {
    forward = -20;
    turn += avoidL > avoidR ? -130 : 130;
  }

  const v = clamp(forward, -40, 110);
  const w = clamp(turn, -3.2, 3.2);

  robotState.vl = clamp(v - (w * ROBOT_BASE.wheelBase) / 2, -ROBOT_BASE.maxWheelSpeed, ROBOT_BASE.maxWheelSpeed);
  robotState.vr = clamp(v + (w * ROBOT_BASE.wheelBase) / 2, -ROBOT_BASE.maxWheelSpeed, ROBOT_BASE.maxWheelSpeed);

  const linear = (robotState.vl + robotState.vr) / 2;
  const angular = (robotState.vr - robotState.vl) / ROBOT_BASE.wheelBase;
  const nx = robotState.x + Math.cos(robotState.theta) * linear * dt;
  const ny = robotState.y + Math.sin(robotState.theta) * linear * dt;
  const nt = robotState.theta + angular * dt;

  if (!blocked(nx, ny, ROBOT_BASE.radius)) {
    robotState.x = nx;
    robotState.y = ny;
    robotState.theta = nt;
  } else {
    robotState.theta += (Math.random() - 0.5) * 0.8;
    robotState.vl *= 0.3;
    robotState.vr *= 0.3;
  }

  const intensity = lightIntensity(robotState.x, robotState.y);
  robotState.bestIntensity = Math.max(robotState.bestIntensity, intensity);
  robotState.trail.push({ x: robotState.x, y: robotState.y });
  if (robotState.trail.length > 900) robotState.trail.shift();
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

function drawTrail() {
  if (robot.trail.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(robot.trail[0].x, robot.trail[0].y);
  for (let i = 1; i < robot.trail.length; i++) {
    ctx.lineTo(robot.trail[i].x, robot.trail[i].y);
  }
  ctx.strokeStyle = "rgba(255, 140, 66, 0.5)";
  ctx.lineWidth = 2;
  ctx.stroke();
}

function drawRobot() {
  const sensors = sense(robot);
  for (const s of sensors) {
    const ex = robot.x + Math.cos(s.angle) * s.distance;
    const ey = robot.y + Math.sin(s.angle) * s.distance;
    ctx.beginPath();
    ctx.moveTo(robot.x, robot.y);
    ctx.lineTo(ex, ey);
    ctx.strokeStyle = "rgba(190, 220, 255, 0.45)";
    ctx.stroke();
  }

  ctx.beginPath();
  ctx.arc(robot.x, robot.y, ROBOT_BASE.radius, 0, Math.PI * 2);
  ctx.fillStyle = "#ff8c42";
  ctx.fill();
  ctx.strokeStyle = "#ffd6b7";
  ctx.lineWidth = 2;
  ctx.stroke();

  const hx = robot.x + Math.cos(robot.theta) * ROBOT_BASE.radius;
  const hy = robot.y + Math.sin(robot.theta) * ROBOT_BASE.radius;
  ctx.beginPath();
  ctx.moveTo(robot.x, robot.y);
  ctx.lineTo(hx, hy);
  ctx.strokeStyle = "#fff";
  ctx.lineWidth = 2.2;
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
  drawTrail();
  drawGoal();
  drawRobot();
}

function updateStats() {
  const intensity = lightIntensity(robot.x, robot.y);
  const dToGoal = Math.hypot(robot.x - light.x, robot.y - light.y);
  statsEl.innerHTML = [
    `Time: ${SIM.time.toFixed(1)} s`,
    `Position: (${robot.x.toFixed(1)}, ${robot.y.toFixed(1)})`,
    `Heading: ${(robot.theta % (Math.PI * 2)).toFixed(2)} rad`,
    `Wheel Speeds (L/R): ${robot.vl.toFixed(1)} / ${robot.vr.toFixed(1)}`,
    `Current Brightness: ${intensity.toFixed(3)}`,
    `Best Brightness: ${robot.bestIntensity.toFixed(3)}`,
    `Distance to Brightest Spot: ${dToGoal.toFixed(1)} px`,
  ].join("<br>");
}

let lastTs = 0;
function loop(ts) {
  if (!lastTs) lastTs = ts;
  const dt = Math.min((ts - lastTs) / 1000, SIM.dtMax);
  lastTs = ts;
  if (SIM.running) {
    const sensors = sense(robot);
    controller(robot, sensors, dt);
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
  resetRobot();
});

newMapBtn.addEventListener("click", () => {
  generateMap();
  resetRobot();
  SIM.time = 0;
});

generateMap();
resetRobot();
requestAnimationFrame(loop);

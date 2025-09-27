const joystick = document.getElementById("joystick");
const stick = document.getElementById("stick");
const maxDistance = 80;
const fireButton = document.getElementById("fire");

let dragging = false;

const DEAD = 0.25;
const SEND_HZ = 15;
let lastSentMs = 0;
let lastCmd = "S";

function send(cmd) {
  if (cmd === lastCmd) return;
  lastCmd = cmd;
  console.log("SEND:", cmd);
}

function chooseCommand(nx, ny) {
  const r = Math.hypot(nx, ny);
  if (r < DEAD) return "S";
  const vertDominates = Math.abs(ny) >= Math.abs(nx);
  if (vertDominates) {
    return ny < 0 ? "F" : "B";
  } else {
    return nx < 0 ? "L" : "R";
  }
}

joystick.addEventListener("pointerdown", () => {
  dragging = true;
});

joystick.addEventListener("pointermove", (e) => {
  if (!dragging) return;

  const rect = joystick.getBoundingClientRect();
  const centerX = rect.left + rect.width / 2;
  const centerY = rect.top + rect.height / 2;

  let dx = e.clientX - centerX;
  let dy = e.clientY - centerY;

  const distance = Math.min(Math.hypot(dx,dy), maxDistance);
  const angle = Math.atan2(dy,dx);

  const x = distance * Math.cos(angle);
  const y = distance * Math.sin(angle);

  stick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;

  const normX = +(x / maxDistance).toFixed(2);
  const normY = +(y / maxDistance).toFixed(2);

  const now = performance.now();
  if (now - lastSentMs >= 1000 / SEND_HZ) {
    const cmd = chooseCommand(normX, normY);
    send(cmd);
    lastSentMs = now;
  }
});

joystick.addEventListener("pointerup", resetStick);
joystick.addEventListener("pointerleave", resetStick);

fireButton.addEventListener("click", () => {
    console.log("FIRE");
})

function resetStick() {
  dragging = false;
  stick.style.transform = "translate(-50%, -50%)";
  send("S");
}

document.addEventListener("visibilitychange", () => {
  if (document.hidden) resetStick();
});


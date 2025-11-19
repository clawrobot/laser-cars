// UI elements
const joystick = document.getElementById("joystick");
const stick = document.getElementById("stick");
const fireButton = document.getElementById("fire");
const speedSlider = document.getElementById("speed")
const maxDistance = 80;
let motorSpeed = 199;

// WebSocket
let ws = null;
let wsConnected = false;

function connectWebSocket() {
  ws = new WebSocket("ws://192.168.4.1/ws");

  ws.onopen = () => {
    wsConnected = true;
    console.log("WebSocket Connected");
    updateConnectionStatus(true);
  };

  ws.onclose = () => {
    wsConnected = false;
    console.log("WebSocket Disconnected");
    updateConnectionStatus(false);
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = (e) => console.error("WS Error", e);
  ws.onmessage = (event) => console.log("ESP32:", event.data);
}

function updateConnectionStatus(status) {
  const el = document.getElementById("isConnected");
  el.textContent = status ? "Connected" : "Disconnected";
  el.className = status ? "status-dot on" : "status-dot off";
}

connectWebSocket();

// ==== Joystick Logic ====

let dragging = false;
const DEAD = 0.25;
const SEND_HZ = 15;
let lastSentMs = 0;
let lastCmd = "S";

function chooseCommand(nx, ny) {
  const r = Math.hypot(nx, ny);
  if (r < DEAD) return "S";
  return Math.abs(ny) >= Math.abs(nx)
    ? (ny < 0 ? "F" : "B")
    : (nx < 0 ? "L" : "R");
}

function send(cmd) {
  if (cmd === lastCmd) return;
  lastCmd = cmd;
  console.log("SEND:", cmd);

  if (wsConnected && ws.readyState === WebSocket.OPEN)
    ws.send(cmd);
}

speedSlider.addEventListener("input", (e) => {
  const sliderVal = parseInt(e.target.value);
  const sliderMin = parseInt(speedSlider.min);
  const sliderMax = parseInt(speedSlider.max);

  //Conversts value to percentage(0-100) -> Motor Speed
  const percentage = ((sliderVal - sliderMin) / (sliderMax - sliderMin))*100;
  motorSpeed = Math.floor((percentage/100)*255);
  
  //Update displays
  speedPercent.textContent = Math.round(percentage);
  motorValue.textContent = motorSpeed;

  //Sends info to webSocket
  if(wsConnected && ws.readyState === WebSocket.OPEN){
    ws.send(`SPEED:${motorSpeed}`);
  }
});

joystick.addEventListener("pointerdown", () => (dragging = true));

joystick.addEventListener("pointermove", (e) => {
  if (!dragging) return;

  const rect = joystick.getBoundingClientRect();
  const dx = e.clientX - (rect.left + rect.width / 2);
  const dy = e.clientY - (rect.top + rect.height / 2);

  const distance = Math.min(Math.hypot(dx, dy), maxDistance);
  const angle = Math.atan2(dy, dx);

  const x = distance * Math.cos(angle);
  const y = distance * Math.sin(angle);

  stick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;

  const nx = +(x / maxDistance).toFixed(2);
  const ny = +(y / maxDistance).toFixed(2);

  const now = performance.now();
  if (now - lastSentMs >= 1000 / SEND_HZ) {
    send(chooseCommand(nx, ny));
    lastSentMs = now;
  }
});

function resetStick() {
  dragging = false;
  stick.style.transform = "translate(-50%, -50%)";
  send("S");
}

joystick.addEventListener("pointerup", resetStick);
joystick.addEventListener("pointerleave", resetStick);

fireButton.addEventListener("click", () => send("FIRE"));

// Safety stop on page unload
window.addEventListener("beforeunload", () => {
  if (wsConnected) ws.send("S");
});

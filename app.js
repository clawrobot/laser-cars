const joystick = document.getElementById("joystick");
const stick = document.getElementById("stick");
const maxDistance = 80;

let dragging = false;

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


    const normX = (x / maxDistance).toFixed(2);
    const normY = (y / maxDistance).toFixed(2);

    console.log("Joystick:", normX, normY);
});

joystick.addEventListener("pointerup", resetStick);
joystick.addEventListener("pointerleave", resetStick);

function resetStick() {
    dragging = false;
    stick.style.transform = "translate(-50%, -50%)";
    console.log("Joystick released:", 0, 0);
}
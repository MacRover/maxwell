import { SwerveModulesList, DriveWheelCanvas } from "./types";

function drawWheel(ctx: CanvasRenderingContext2D, wheel: DriveWheelCanvas, isLeftSide: boolean) {
  ctx.save();
  ctx.translate(wheel.x, wheel.y);
  ctx.rotate(wheel.angle * Math.PI / 180);
  ctx.fillStyle = wheel.colour;
  ctx.globalAlpha = wheel.alpha;
  ctx.fillRect(-wheel.wheelWidth / 2, -wheel.wheelHeight / 2, wheel.wheelWidth, wheel.wheelHeight);
  if (isLeftSide) {
    ctx.fillRect(-wheel.wheelWidth  / 2, wheel.wheelHeight / 8, -10, -wheel.wheelHeight / 4);
  } else {
    ctx.fillRect(wheel.wheelWidth  / 2, -wheel.wheelHeight / 8, 10, wheel.wheelHeight / 4);
  }
  ctx.restore();
}

export async function drawOnCanvas(canvas: HTMLCanvasElement, modulesCommandMsg: SwerveModulesList, modulesOdomMsg: SwerveModulesList) {
  const ctx = canvas.getContext("2d");

  const wHeight = canvas.height / 5;
  const wWidth = canvas.width / 10;

  function draw() {
    if (!ctx) {
      return;
    }

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    ctx.strokeStyle = "blue";
    ctx.lineWidth = 5;
    ctx.moveTo(canvas.width / 4, canvas.height / 4);
    ctx.lineTo(3 * canvas.width / 4, canvas.height / 4);
    ctx.moveTo(canvas.width / 4, 3 * canvas.height / 4);
    ctx.lineTo(3 * canvas.width / 4, 3 * canvas.height / 4);
    ctx.moveTo(canvas.width / 2, canvas.height / 4);
    ctx.lineTo(canvas.width / 2, 3 * canvas.height / 4);
    ctx.stroke();

    for (let i = 0; i < 4; i++) {
      const x = canvas.width / 2 * (i & 1) + canvas.width / 4;
      const y = canvas.height / 2 * ((i & 2) >> 1) + canvas.height / 4;
      let angleCmd = 0;
      let angleOdom = 0;
      if (i == 0) {
        angleCmd = modulesCommandMsg.front_left.angle;
        angleOdom = modulesOdomMsg.front_left.angle;
      } 
      else if (i == 1) {
        angleCmd = modulesCommandMsg.front_right.angle;
        angleOdom = modulesOdomMsg.front_right.angle;
      }
      else if (i == 2) {
        angleCmd = modulesCommandMsg.rear_left.angle;
        angleOdom = modulesOdomMsg.rear_left.angle;
      }
      else {
        angleCmd = modulesCommandMsg.rear_right.angle;
        angleOdom = modulesOdomMsg.rear_right.angle;
      }
      drawWheel(ctx, {angle: angleCmd, x: x, y: y, colour: "green", alpha: 1.0, wheelWidth: wWidth, wheelHeight: wHeight}, i % 2 == 0);
      drawWheel(ctx, {angle: angleOdom, x: x, y: y, colour: "red", alpha: 0.5, wheelWidth: wWidth, wheelHeight: wHeight}, i % 2 == 0);
    }
  }
  requestAnimationFrame(draw);
}
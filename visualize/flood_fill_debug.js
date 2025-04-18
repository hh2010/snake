let dark = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;

function lookup_grid(data, i) {
  let d = data.charCodeAt(~~(i/6))-35;
  return (d & (1 << (i%6))) != 0;
}

function render_legend() {
  let scale = 20;
  // snake
  {
    let ctx = document.getElementById("legend-snake").getContext("2d");
    ctx.beginPath();
    ctx.lineWidth = 0.4 * scale;
    ctx.strokeStyle = dark ? "#0b0" : "#0a0";
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.moveTo(0.5*scale, 0.5*scale);
    ctx.lineTo(1.5*scale, 0.5*scale);
    ctx.stroke();
  }
  // apple
  {
    let ctx = document.getElementById("legend-apple").getContext("2d");
    ctx.beginPath();
    ctx.arc(scale, 0.5*scale, 0.4*scale, 0, 2 * Math.PI);
    ctx.fillStyle = dark ? "#f00" : "#e00";
    ctx.fill();
  }
  // path
  {
    let ctx = document.getElementById("legend-path").getContext("2d");
    ctx.beginPath();
    ctx.moveTo(0.5*scale, 0.5*scale);
    ctx.lineTo(1.5*scale, 0.5*scale);
    ctx.lineWidth = 0.2 * scale;
    ctx.strokeStyle = dark ? "#04f5" : "#35f5";
    ctx.stroke();
  }
  // fill
  {
    let ctx = document.getElementById("legend-fill").getContext("2d");
    ctx.beginPath();
    ctx.rect(0.4*scale, 0.2*scale, 1.2*scale, 0.6*scale);
    ctx.fillStyle = dark ? "#ff03" : "#ff05";
    ctx.fill();
  }
  // after snake
  {
    let ctx = document.getElementById("legend-after-snake").getContext("2d");
    ctx.beginPath();
    ctx.lineWidth = 0.4 * scale;
    ctx.strokeStyle = dark ? "#a0c" : "#90b";
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.moveTo(0.5*scale, 0.5*scale);
    ctx.lineTo(1.5*scale, 0.5*scale);
    ctx.stroke();
  }
}

function render(game, currentStep, showAfterSnake) {
  let ctx = document.getElementById("canvas").getContext("2d");
  let scale = (ctx.canvas.width-2) / game.size[0];
  ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
  ctx.save();
  ctx.translate(1,1);
  
  // draw grid
  ctx.strokeStyle = dark ? "#666" : "#ccc";
  ctx.lineWidth = 1;
  for (let x=0; x<=game.size[0]; ++x) {
    ctx.beginPath();
    ctx.moveTo(x*scale, 0);
    ctx.lineTo(x*scale, game.size[1]*scale);
    ctx.stroke();
  }
  for (let y=0; y<=game.size[1]; ++y) {
    ctx.beginPath();
    ctx.moveTo(0, y*scale);
    ctx.lineTo(game.size[0]*scale, y*scale);
    ctx.stroke();
  }

  // Fill all cells with yellow initially (unreachable)
  ctx.beginPath();
  for (let y = 0; y < game.size[1]; y++) {
    for (let x = 0; x < game.size[0]; x++) {
      ctx.rect(x*scale+1, y*scale+1, scale-2, scale-2);
    }
  }
  ctx.fillStyle = "#ff03"; // Same yellow as main.js
  ctx.fill();

  // draw flood fill state (grey cells for reachable area)
  if (game.flood_fill_debug.fill_states[currentStep]) {
    ctx.beginPath();
    let grid = game.flood_fill_debug.fill_states[currentStep];
    for (let y=0,i=0; y<game.size[1]; ++y) {
      for (let x=0; x<game.size[0]; ++x,++i) {
        if (lookup_grid(grid, i)) {
          ctx.rect(x*scale+1, y*scale+1, scale-2, scale-2);
        }
      }
    }
    ctx.fillStyle = dark ? "#222" : "#fff"; // Same grey as main.js grid cells
    ctx.fill();
  }

  // Only draw the regular snake and path when not showing the after snake
  if (!showAfterSnake) {
    // draw path to apple
    if (game.flood_fill_debug.plans && game.flood_fill_debug.plans.length > 0) {
      ctx.beginPath();
      let path = game.flood_fill_debug.plans[0];
      ctx.moveTo((path[0][0]+0.5)*scale, (path[0][1]+0.5)*scale);
      for (let i=1; i<path.length; ++i) {
        ctx.lineTo((path[i][0]+0.5)*scale, (path[i][1]+0.5)*scale);
      }
      ctx.lineWidth = 0.2 * scale;
      ctx.strokeStyle = dark ? "#04f5" : "#35f5";
      ctx.stroke();
    }

    // draw snake body
    if (game.snake_pos.length > 1) {
      ctx.beginPath();
      let pos = game.snake_pos;
      // Start from first non-head position and connect the body parts
      ctx.moveTo((pos[1][0]+0.5)*scale, (pos[1][1]+0.5)*scale);
      for (let i=2; i<pos.length; ++i) {
        ctx.lineTo((pos[i][0]+0.5)*scale, (pos[i][1]+0.5)*scale);
      }
      // Only connect the head to the cell before it
      ctx.moveTo((pos[1][0]+0.5)*scale, (pos[1][1]+0.5)*scale);
      ctx.lineTo((pos[0][0]+0.5)*scale, (pos[0][1]+0.5)*scale);
      
      ctx.strokeStyle = dark ? "#0b0" : "#0a0";
      ctx.lineWidth = 0.4 * scale;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.stroke();
    }

    // draw snake head as circle
    {
      let head = game.snake_pos[0];
      ctx.beginPath();
      ctx.arc((head[0]+0.5)*scale, (head[1]+0.5)*scale, 0.4*scale, 0, 2 * Math.PI);
      ctx.fillStyle = dark ? "#0b0" : "#0a0";
      ctx.fill();
    }
  } 
  // Draw after snake path if toggled on and it exists
  else if (game.flood_fill_debug.after_snake && game.flood_fill_debug.after_snake.length > 0) {
    ctx.beginPath();
    let path = game.flood_fill_debug.after_snake[0];
    ctx.moveTo((path[0][0]+0.5)*scale, (path[0][1]+0.5)*scale);
    for (let i=1; i<path.length; ++i) {
      ctx.lineTo((path[i][0]+0.5)*scale, (path[i][1]+0.5)*scale);
    }
    // Draw the after snake in purple to distinguish it
    ctx.strokeStyle = dark ? "#a0c" : "#90b";
    ctx.lineWidth = 0.4 * scale;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.stroke();
    
    // Draw the head of after snake as a circle
    let head = path[0];
    ctx.beginPath();
    ctx.arc((head[0]+0.5)*scale, (head[1]+0.5)*scale, 0.4*scale, 0, 2 * Math.PI);
    ctx.fillStyle = dark ? "#a0c" : "#90b";
    ctx.fill();
  }

  // draw apple (always shown)
  {
    ctx.beginPath();
    ctx.arc((game.apple_pos[0]+0.5)*scale, (game.apple_pos[1]+0.5)*scale, 0.4*scale, 0, 2 * Math.PI);
    ctx.fillStyle = dark ? "#f00" : "#e00";
    ctx.fill();
  }

  ctx.restore();
}

function init() {
  let currentStep = 0;
  let stepSize = 1;
  let game;
  let showAfterSnake = false;

  function updateUI() {
    document.getElementById("turn").innerText = "Turn: " + game.turn;
    document.getElementById("size").innerText = "Size: " + game.snake_size;
    document.getElementById("fill-step").innerText = currentStep + " (Step Size: " + stepSize + ")";
    document.getElementById("view-mode").innerText = showAfterSnake ? "View Mode: After Snake" : "View Mode: Regular";
  }

  function nextStep() {
    if (!game) return;
    currentStep = Math.min(currentStep + stepSize, game.flood_fill_debug.fill_states.length - 1);
    render(game, currentStep, showAfterSnake);
    updateUI();
  }

  function prevStep() {
    if (!game) return;
    currentStep = Math.max(currentStep - stepSize, 0);
    render(game, currentStep, showAfterSnake);
    updateUI();
  }

  function increaseStepSize() {
    stepSize = Math.min(stepSize * 2, 32);
    updateUI();
  }

  function decreaseStepSize() {
    stepSize = Math.max(stepSize / 2, 1);
    updateUI();
  }

  function toggleAfterSnake() {
    showAfterSnake = !showAfterSnake;
    render(game, currentStep, showAfterSnake);
    updateUI();
  }

  document.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowRight') nextStep();
    else if (e.key === 'ArrowLeft') prevStep();
    else if (e.key === 'ArrowUp') increaseStepSize();
    else if (e.key === 'ArrowDown') decreaseStepSize();
    else if (e.key === ' ' || e.code === 'Space') toggleAfterSnake();
  });

  document.getElementById('btn_next_step').onclick = nextStep;
  document.getElementById('btn_prev_step').onclick = prevStep;
  document.getElementById('btn_speed_up').onclick = increaseStepSize;
  document.getElementById('btn_speed_down').onclick = decreaseStepSize;
  document.getElementById('btn_toggle_after_snake').onclick = toggleAfterSnake;

  // Load game data from URL parameter
  let params = new URLSearchParams(window.location.search);
  let dataFile = params.get('f');
  if (dataFile) {
    fetch(dataFile)
      .then(response => response.json())
      .then(data => {
        game = data;
        render(game, currentStep, showAfterSnake);
        updateUI();
      });
  }

  render_legend();
}

init();
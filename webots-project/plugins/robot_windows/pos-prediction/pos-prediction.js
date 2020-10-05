let randomMovingMode = true;

function loadChartParticles(items) {
    let plot = document.getElementById('plot');
    let xy = {
        x: items.x,
        y: items.y,
        name: 'Real Position',
        mode: 'lines',
          line: {
            width: 1.5
          }
    }
    
    let xy_particles = {
        x: items.particles_x,
        y: items.particles_y,
        name: 'Particles',
        mode: 'markers',
        marker: {
            size: 3
        }
    }
    
    let xy_pred = {
        x: items.x_pred,
        y: items.y_pred,
        name: 'Particles Prediction',
         mode: 'markers',
         marker: {
            size: 4
        }
    }
    
     let xy_odometry = {
        x: items.x_odometry,
        y: items.y_odometry,
        name: 'Odometry',
         mode: 'lines',
          line: {
            width: 1.5
          }
    }

    let data = [
        xy,
        xy_particles,
        xy_odometry,
        xy_pred
    ]

    var layout = {
        title: 'Real Position, Particles, Odometry',
        width: 800,
        height: 600,
        margin: {
            l: 30,
            r: 30,
            b: 30,
            t: 30,
            pad: 4
          },
        xaxis: {
            range: [0, 3],
            autorange: false
        },
        yaxis: {
            range: [0, 3],
            autorange: false
        },
      };

    Plotly.newPlot(plot, data, layout);
}

function modifyMovementRandomness() {
    randomMovingMode = !randomMovingMode;
    let joystick = document.getElementById('joystick');

    if (randomMovingMode) {
        window.robotWindow.send(JSON.stringify({code: "start_randomness"}));
        joystick.hidden = true;
    } else {
        window.robotWindow.send(JSON.stringify({code: "stop_randomness"}));
        joystick.hidden = false;
    }
}

function move(direction) {
    window.robotWindow.send(JSON.stringify({
        code: "move",
        direction: direction
    }));
}

function setParams() {
    // check that all the params are filled
    let numParticles = document.getElementById("numParticles").value;
    let sigmaXy = document.getElementById("sigmaXy").value;
    let sigmaTheta = document.getElementById("sigmaTheta").value;
    let correct = true;
    
    if (!numParticles || (numParticles && isNaN(numParticles) || (numParticles && numParticles <= 0))) {
        document.getElementById("errorNumParticles").style.display = "inline";
        correct = false;
    } else {
        document.getElementById("errorNumParticles").style.display = "none";
    }
    
    if (!sigmaXy || (sigmaXy && isNaN(sigmaXy) || (sigmaXy && sigmaXy <= 0))) {
        document.getElementById("errorSigmaXy").style.display = "inline";
        correct = false
    } else {
        document.getElementById("errorSigmaXy").style.display = "none";
    }
    
    if (!sigmaTheta || (sigmaTheta && isNaN(sigmaTheta) || (sigmaTheta && sigmaTheta <= 0))) {
        document.getElementById("errorSigmaTheta").style.display = "inline";
        correct = false
    } else {
        document.getElementById("errorSigmaTheta").style.display = "none";
    }
    
    if (!correct) return;
    
    // send params to controller
    window.robotWindow.send(JSON.stringify({
        code: "params_modification",
        number_particles: numParticles,
        sigma_xy: sigmaXy,
        sigma_theta: sigmaTheta
    }));
    
}

function init(data) {
    document.getElementById("numParticles").value = data.num_particles;
    document.getElementById("sigmaXy").value = data.sigma_xy;
    document.getElementById("sigmaTheta").value = data.sigma_theta;
}

window.onload = function() {
  window.robotWindow = webots.window();
  window.robotWindow.receive = function(value, robot) {
        let data = JSON.parse(value);
        if (data.items) {
            loadChartParticles(data.items)
        }
        if (data.code === 'init') {
            init (data);
        }
  }
  loadChartParticles([]);
  document.getElementById('joystick').hidden = true;
}


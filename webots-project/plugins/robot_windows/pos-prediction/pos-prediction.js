let randomMovingMode = true;

// Log a message in the console widget.
function log(message) {
  var ul = document.getElementById('console');
  var li = document.createElement('li');
  li.appendChild(document.createTextNode(message));
  ul.appendChild(li);
}

function loadChart(items) {
    let plot = document.getElementById('plot');
    let xy = {
        x: items.x,
        y: items.y,
        name: 'Real Position'
    }

    let xy_odometry = {
        x: items.x_odometry,
        y: items.y_odometry,
        name: 'Odometry'
    }
    
    let xy_pred = {
        x: items.x_pred,
        y: items.y_pred,
        name: 'Prediction',
        mode: 'markers'
    }

    let data = [
        xy,
        xy_odometry,
        xy_pred
    ]

    var layout = {
        title: 'Real Position vs Odometry',
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
            range: [0, 9],
            autorange: false
        },
        yaxis: {
            range: [0, 7],
            autorange: false
        },
      };

	Plotly.newPlot(plot, data, layout);
}

function loadChartParticles(items) {
    let plot = document.getElementById('plot');
    let xy = {
        x: items.x,
        y: items.y,
        name: 'Real Position'
    }
    
    let xy_pred = {
        x: items.particles_x,
        y: items.particles_y,
        name: 'Particles',
        mode: 'markers'
    }

    let data = [
        xy,
        xy_pred
    ]

    var layout = {
        title: 'Real Position vs Odometry',
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
            range: [0, 9],
            autorange: false
        },
        yaxis: {
            range: [0, 7],
            autorange: false
        },
      };

    Plotly.newPlot(plot, data, layout);
}

function modifyMovementRandomness() {
    randomMovingMode = !randomMovingMode;
    let joystick = document.getElementById('joystick');

    if (randomMovingMode) {
        window.robotWindow.send('start_randomness');
        joystick.hidden = true;
    } else {
        window.robotWindow.send('stop_randomness');
        joystick.hidden = false;
    }
}

function move(direction) {
    window.robotWindow.send(direction);
}

window.onload = function() {
  window.robotWindow = webots.window();
  window.robotWindow.receive = function(value, robot) {
        let data = JSON.parse(value);
        loadChartParticles(data.items)
  }
  loadChartParticles([]);
  document.getElementById('joystick').hidden = true;
}


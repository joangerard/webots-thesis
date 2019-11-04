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
        width: 500,
        height: 500,
        xaxis: {
            range: [0, 2],
            autorange: false
        },
        yaxis: {
            range: [0, 2],
            autorange: false
        },
      };

	Plotly.newPlot('plot', data, layout);
}

window.onload = function() {
  window.robotWindow = webots.window();
  window.robotWindow.receive = function(value, robot) {
        let data = JSON.parse(value);
        loadChart(data.items)
  }
  loadChart([]);
}


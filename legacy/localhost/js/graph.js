function ActuatorToGraph() {   
	this.time = new Array(0);
	this.angularPosition = new Array(0);
	this.angularVelocity = new Array(0);
	this.angularAcceleration = new Array(0);
	this.applicationTorque = new Array(0);
	
	this.displayData = function(){
		var hudID = document.getElementById("hud");
		hudID.innerHTML = '<div><b>Torque data</b></br>RMS: ' + calculateRMS(this.applicationTorque) + ' Nm</br>Max.: ' + math.max(this.applicationTorque) + ' Nm</br>Min.: ' + math.min(this.applicationTorque) + ' Nm</div></br>'; 
		hudID.innerHTML = hudID.innerHTML + '<canvas id="positionChart" width="400" height="400" style="background-color: #FFFFFF;"></canvas></br></br>';
		hudID.innerHTML = hudID.innerHTML + '<canvas id="velocityChart" width="400" height="400" style="background-color: #FFFFFF;"></canvas></br></br>';
		hudID.innerHTML = hudID.innerHTML + '<canvas id="accelerationChart" width="400" height="400" style="background-color: #FFFFFF;"></canvas></br></br>';
		hudID.innerHTML = hudID.innerHTML + '<canvas id="torqueChart" width="400" height="400" style="background-color: #FFFFFF;"></canvas></br></br>';
		hudID.innerHTML = hudID.innerHTML + '<canvas id="loadChart" width="400" height="400" style="background-color: #FFFFFF;"></canvas></br></br>';
				
		var positionID = document.getElementById("positionChart");
		var velocityID = document.getElementById("velocityChart");
		var accelerationID = document.getElementById("accelerationChart");
		var torqueID = document.getElementById("torqueChart");
		var loadID = document.getElementById("loadChart");
		
		var positionChart = new Chart(positionID, {
		  type: 'line',
		  data: {
			labels: this.time,
			datasets: [
			  { 
				label: 'Angular position [rad]',
				data: this.angularPosition,
				fill: false,
				borderColor: '#2196f3', // Add custom color border (Line)
				backgroundColor: '#2196f3', // Add custom color background (Points and Fill)
				borderWidth: 1,
				radius: 0				
			  }
			]
		  },
		  options: {
		  }
		});
		
		var velocityChart = new Chart(velocityID, {
		  type: 'line',
		  data: {
			labels: this.time,
			datasets: [
			  { 
				label: 'Angular velocity [rad/s]',
				data: this.angularVelocity,
				fill: false,
				borderColor: '#2196f3', // Add custom color border (Line)
				backgroundColor: '#2196f3', // Add custom color background (Points and Fill)
				borderWidth: 1,
				radius: 0
			  }
			]
		  },
		  options: {
			  
		  }
		});
		
		var accelerationChart = new Chart(accelerationID, {
		  type: 'line',
		  data: {
			labels: this.time,
			datasets: [
			  { 
				label: 'Angular acceleration [rad/s^2]',
				data: this.angularAcceleration,
				fill: false,
				borderColor: '#2196f3', // Add custom color border (Line)
				backgroundColor: '#2196f3', // Add custom color background (Points and Fill)
				borderWidth: 1,
				radius: 0
			  }
			]
		  },
		  options: {
		  }
		});
		
		var torqueChart = new Chart(torqueID, {
		  type: 'line',
		  data: {
			labels: this.time,
			datasets: [
			  { 
				label: 'Application torque [Nm]',
				data: this.applicationTorque,
				fill: false,
				borderColor: '#2196f3', // Add custom color border (Line)
				backgroundColor: '#2196f3', // Add custom color background (Points and Fill)
				borderWidth: 1,
				radius: 0
			  }
			]
		  },
		  options: {
		  }
		});
		
		const data = this.angularVelocity.map((x, i) => {
			return {
				x: Math.abs(x * 60 / (2 * Math.PI)),
				y: Math.abs(this.applicationTorque[i])
			};
		});

		var loadChart = new Chart(loadID, {
		  type: 'scatter',
		  data: {
			datasets: [
				{
					data: [{
						x: (calculateAvg(this.angularVelocity) * 60 / (2 * Math.PI)),
						y: calculateRMS(this.applicationTorque)
					}],
					label: 'RMS',
					backgroundColor: 'rgba(255, 0, 0, 1)',
				},
				{ 
					label: 'Load characteristic',
					data: data,
					borderColor: '#2196f3', // Add custom color border (Line)
					backgroundColor: '#2196f3', // Add custom color background (Points and Fill)
				}
			]
		  },
		});
	}
}

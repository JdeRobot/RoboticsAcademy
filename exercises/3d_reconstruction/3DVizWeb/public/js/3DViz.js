let config = {};
var w;
var lineInterval, pointInterval;
try{
	const yaml = require('js-yaml');
	const fs = require('fs');
	config = yaml.safeLoad(fs.readFileSync('public/config.yml', 'utf8'))
} catch (e) {
	config.Server = "localhost";
	config.Port = "11000";
	config.updatePoints= 10
	config.updateSegments= 10
	config.linewidth= 2
	config.pointsize= 8
	config.camera = {}
	config.camera.x = 100
	config.camera.y = 50
	config.camera.z = 300

}
    function startWorker(){
      if(typeof(Worker) !== "undefined") {
        if(typeof(w) == "undefined") {
          w = new Worker("js/3DViz_worker.js");
          w.postMessage({func:"Start",server:config.Server, port:config.Port});
      }
      } else {
        Console.log("Sorry, your browser does not support Web Workers...");
      }
      w.onmessage = function(event) {
        if (event.data.func == "Connect"){
					setPlane();
        } else {
          console.log(event.data);
          w.terminate();
        }
      }
    }

    function stop(){
      if(typeof(w) != "undefined") {
        w.postMessage({func:"Stop"});
        w.onmessage = function(event) {
					if (event.data.func == "Disconnect"){
            w.terminate();
            w = undefined;
      }
    }}
    }

		function setPlane(){
			w.postMessage({func:"setLine"});
			w.onmessage = function(event){
				if (event.data.segments.buffer.length > 0){
						segments = event.data.segments.buffer;
						for (var i = 0; i < segments.length; i+=1) {
		        	addLine(segments[i], "plane");
						}
			} pointInterval = setInterval(function(){
				setPoint();
			},config.updatePoints);
			lineInterval = setInterval(function(){
				setLine();
			},config.updateSegments);}
		}

		function setLine(){
			w.postMessage({func:"setLine"});
			getData();
		}

    function setPoint(){
      w.postMessage({func:"setPoint"});
			getData();
}
		function getData (){
			w.onmessage = function(event) {
				if (event.data.func == "drawLine"){
					if (event.data.segments.refresh & (event.data.segments.buffer.length !=0)){
						deleteObj("segments");
					}
						segments = event.data.segments.buffer;
						for (var i = 0; i < segments.length; i+=1) {
		        	addLine(segments[i], "segments");
						}
				} else if (event.data.func == "drawPoint"){
					if (event.data.points.refresh & (event.data.points.buffer.length != 0)){
						deleteObj("points");
					}
					points = event.data.points.buffer;
				for (var i = 0; i < points.length; i+=1) {
        	addPoint(points[i]);
				}
				}
			}
		}

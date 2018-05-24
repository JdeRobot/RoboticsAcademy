let config = {};
var w;
try{
	const yaml = require('js-yaml');
	const fs = require('fs');
	config = yaml.safeLoad(fs.readFileSync('public/config.yml', 'utf8'))
} catch (e) {
	config.Server = "localhost";
	config.Port = "11000";
}
    function startWorker(){
      if(typeof(Worker) !== "undefined") {
        if(typeof(w) == "undefined") {
          w = new Worker("js/3DViewer_worker.js");
          w.postMessage({func:"Start",server:config.Server, port:config.Port});
      }
      } else {
        Console.log("Sorry, your browser does not support Web Workers...");
      }
      w.onmessage = function(event) {
        if (event.data.func == "Connect"){
          console.log(event.data.func);
					setLine();
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

		function setLine(){
			w.postMessage({func:"setLine"});
			w.onmessage = function(event){
				if (event.data.length > 0){
						segments = event.data;
						for (var i = 0; i < segments.length; i+=1) {
		        	addLine(segments[i]);
						}
						setPoint();
			}}
		}

    function setPoint(){
      w.postMessage({func:"setPoint"});
      w.onmessage = function(event) {
				if (event.data.func == "drawPoint"){
					points = event.data.points;
				for (var i = 0; i < points.length; i+=1) {
        	addPoint(points[i]);
				}
				setInterval(setPoint(),1000);
    }}
    }

    function clearAll(){
      if(typeof(w) == "undefined") {
        deleteObj();
        console.log("Clear all");
        startWorker();
      } else {
        w.postMessage({func:"ClearAll"});
        w.onmessage = function(event) {
          if (event.data.func == "ClearAll"){
          if(typeof(w) != "undefined") {
            w.terminate();
            w = undefined;
          }
          deleteObj();
          startWorker();
        }}
    }
  }

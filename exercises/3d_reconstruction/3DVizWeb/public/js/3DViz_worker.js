// variables to make believe to ICE that is in the main thread
var window=self;
var global=self;

// importing required files
importScripts('jderobot/Ice.min.js');
importScripts('jderobot/datetime.js');
importScripts('jderobot/exceptions.js');
importScripts('jderobot/containers.js');
importScripts('jderobot/common.js');
importScripts('jderobot/image.js');
importScripts('jderobot/primitives.js');
importScripts('jderobot/visualization.js')
importScripts('jderobot/camera.js');

// variables related to the configuration and connection of ICE
var ic = Ice.initialize();
var communicator;
var Promise;
var Prx = jderobot.VisualizationPrx;
var srv;

function connect(server,port){
  endpoint = "ws -h " + server + " -p " + port;
  var proxy = ic.stringToProxy("3DViz:" + endpoint);
  Promise = Prx.checkedCast(proxy).then(
      function(printer)
      {
          srv = printer;
          self.postMessage({func:"Connect"});
      });
}

function setPoint(point){
  srv.getPoints().then(function(data){
    self.postMessage({func:"drawPoint",points: data});
  });
}

function setLine(){
  srv.getSegment().then(function(data){
      self.postMessage({func:"drawLine", segments: data});
  });
}

function clearAll(){
  srv.clearAll().then(function(data){
    console.log("Clear all");
    self.postMessage({func:"ClearAll"});
  });
}

function disconnect(){
    console.log("Disconnect");
    srv = undefined;
    self.postMessage({func:"Disconnect"});
}

onmessage = function(e) {

      switch (e.data.func){
      case "Start":
            var server = e.data.server;
            var port = e.data.port;
            connect(server,port);
            break;
      case "Stop":
            disconnect();
            break;
      case "ClearAll":
            clearAll();
            break;
      case "setLine":
            setLine();
            break;
      case "setPoint":
            var point = []
            setPoint(point);
            break;
      }
}

// WebRTC file

// The stream & capture
var stream = document.getElementById('stream');
var streaming = false;

// The video stream
var cameraStream = null;
var vc = null;

var width = 320
var src = null;
var ack= true;
var last_img = null;
var last_img_no_send = false;
//WebSocket for Code
var websocket_webrtcframe;

function declare_webrtcframe(){
    websocket_webrtcframe = new WebSocket("ws://" + websocket_address + ":1831/");

    websocket_webrtcframe.onopen = function(event){
        alert("[open] Connection established!");
    }
    websocket_webrtcframe.onclose = function(event){
        if(event.wasClean){
            alert(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
        }
        else{
            alert("[close] Connection closed webrtc!");
        }
    }

    websocket_webrtcframe.onmessage = function(event){
        var source_img = event.data;
        operation = source_img.substring(0, 5);

        if(operation == "#load"){
            editor.setValue(source_img.substring(5,));
        }
        else if(operation == "#freq"){
            frequency = source_img.substring(5,);
            document.querySelector('#ideal_frequency').value = frequency;
        }
        else if(operation == "#ping"){
            websocket_webrtcframe.send("#pong")
        }
		else if(operation == "#ack"){
			if(last_img_no_send == true)
			{
				last_img_no_send = false;
				send_image(last_img);
			}else
				ack = true;
        }
    };
}


// Start Streaming
function startStreaming() {

    var mediaSupport = 'mediaDevices' in navigator;

    if (mediaSupport && null == cameraStream) {

        navigator.mediaDevices.getUserMedia({video: true})
        .then(function(mediaStream) {

            cameraStream = mediaStream;
            stream.srcObject = mediaStream;
            stream.play();
        })
        .catch(function(err) {

            console.log("Unable to access camera: " + err);
        });
    }
    else {

        alert('Your browser does not support media devices.');

        return;
    }

    stream.addEventListener("canplay", function(ev){
        if (!streaming){
            streaming = true;
            height = stream.videoHeight / (stream.videoWidth/width);
            stream.setAttribute("width", width);
            stream.setAttribute("height", height);
            vc = new cv.VideoCapture(stream);
        }

        startVideoProcessing();
    }, false);
}

// Functions to control video processing
function startVideoProcessing() {
    if (!streaming) {
        console.warn("Please startup your webcam");
        return;
    }

    stopVideoProcessing();
    src = new cv.Mat(height, width, cv.CV_8UC4);
    requestAnimationFrame(processVideo);
    startSendImage();
}

function stopVideoProcessing() {
    if (src != null && !src.isDeleted())
    {
        src.delete();
    }
}

function processVideo() {
    if (streaming) {
        vc.read(src);
        requestAnimationFrame(processVideo);
    }
}

function encode_utf8(s){
    return encodeURI((s))
}

function send_image(image){
	let dst2 = new cv.Mat(240, 320, cv.CV_8UC3);
	cv.cvtColor(image, dst2, cv.COLOR_BGRA2BGR);
    img = encode_utf8(dst2.data);

    websocket_webrtcframe.send(img);
	dst2.delete();
}

function startSendImage(){
    setInterval(function(){
        if (streaming != null){
			if(ack == true)
            {
				send_image(src);
				ack = false;
			}else
			{
				last_img = src;
				last_img_no_send = true;
			}
        }
    }, 100);

}


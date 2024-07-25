
// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

let image = new Image();
let image_camera = new Image();
// The stream & capture
//var stream = document.getElementById('stream');
// The video stream
//var cameraStream = null;

export function drawImage (data){
    var canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d')

    // For image object
    image.onload = function(){
        update_image();
    }

    // Request Animation Frame to remove the flickers
    function update_image(){
        window.requestAnimationFrame(update_image);
        context.drawImage(image, 0, 0);
    }
            
    // Parse the Image Data
    var image_data = JSON.parse(data.image),
        source = decode_utf8(image_data.image),
        shape = image_data.shape;
    
    if(source != ""){
        image.src = "data:image/jpeg;base64," + source;
        canvas.width = shape[1];
        canvas.height = shape[0];
    }   
}


// Start Streaming
/*export function startStreaming() {

console.log("startStreaming");
    var mediaSupport = 'mediaDevices' in navigator;

    if( mediaSupport && null == cameraStream ) {

        window.navigator.mediaDevices.getUserMedia({video: true})
        .then(function(mediaStream) {

            cameraStream = mediaStream;
            videoRef.current.srcObject = mediaStream;
            videoRef.current.play();
            console.log("stream play");

        })
        .catch(function(err) {

            console.log("Unable to access camera: " + err);
            //stopStreaming();
        });
    }
    else {

        alert('Your browser does not support media devices.');

        return;
    }
    //requestAnimationFrame(showImageOutput);

}*/

/*function stopStreaming() {

    if(null != cameraStream) {

        var track = cameraStream.getTracks()[0];

        track.stop();
        stream.load();

        cameraStream = null;
    }
}*/

// To decode the image string we will receive from server
function decode_utf8(s) {
    return decodeURIComponent(escape(s));
}

let image = new Image();

export function drawImage(data) {
    console.log("Drawing");
    const canvas = document.getElementById("canvas"),
        context = canvas.getContext("2d");

    // For image object
    image.onload = function () {
        update_image();
    };

    // Request Animation Frame to remove the flickers
    function update_image() {
        window.requestAnimationFrame(update_image);
        // Clean canvas
        context.clearRect(0, 0, canvas.width, canvas.height);
        context.drawImage(image, 0, 0);

        if (data.lapTime) {
            drawLapTime(context, data.lapTime);
        }

        // Not implemented
        if (data.progress) {
            drawProgressBar(context, data.progress);
        }
    }

    // Parse the Image Data
    const image_data = JSON.parse(data.image),
        source = decode_utf8(image_data.image),
        shape = image_data.shape;

    if (source != "") {
        image.src = "data:image/jpeg;base64," + source;
        canvas.width = shape[1];
        canvas.height = shape[0];
    }
}

function parseLapTime(lapTime) {
    const [hours, minutes, secondsMillis] = lapTime.split(':');
    const [seconds, millis] = secondsMillis.split('.');

    const totalMinutes = parseInt(minutes, 10) || 0;
    const totalSeconds = parseInt(seconds, 10) || 0;
    const milliseconds = parseInt(millis.slice(0, 2), 10) || 0;
    return (totalMinutes * 60 + totalSeconds) * 1000 + milliseconds;
}

function drawLapTime(ctx, lapTime) {
    const lapTimeInMillis = parseLapTime(lapTime);

    const totalSeconds = lapTimeInMillis / 1000;

    const minutes = Math.floor(totalSeconds / 60);
    const seconds = Math.floor(totalSeconds % 60);
    const milliseconds = lapTimeInMillis-(minutes*60*1000+seconds*1000);

    const formattedTime = `${minutes}:${seconds.toString().padStart(2, '0')}.${milliseconds.toString().padStart(2, '0')}`;

    ctx.clearRect(ctx.canvas.width - 100, 16, 100, 30);

    ctx.font = "bold 24px sans-serif";
    ctx.fillStyle = "red";
    ctx.textAlign = "right";
    ctx.textBaseline = "top";
    ctx.fillText(formattedTime, ctx.canvas.width - 16, 16);
}


// Not implemented
function drawProgressBar(ctx, progress) {
    return
}

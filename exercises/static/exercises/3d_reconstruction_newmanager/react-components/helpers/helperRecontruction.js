// Set the src of iframe tag
var matching_history = [];
var matching_history_color = [];
var paint_matching = 'false';

// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s));
}

// Websocket and other variables for image display
var image_data1, image_data2, source1, source2
let image1 = new Image();
let image2 = new Image();
image1.onload = function(){
    update_image();
}

image2.onload = function(){
    update_image();
}

export function draw (data){
            image_data1 = JSON.parse(data.img1)
            source1 = decode_utf8(image_data1.img)
            if(data.img1 !== null && data.img1 !== undefined){
                image_data1 = JSON.parse(data.img1)
                source1 = decode_utf8(image_data1.img)
                if(source1.length > 10){
                    image1.src = "data:image1/jpeg;base64," + source1;
                    
                }
            }
            
            if(data.img2 !== null && data.img2 !== undefined){
                image_data2 = JSON.parse(data.img2)
                source2 = decode_utf8(image_data2.img)
                if(source2.length > 10){
                    image2.src = "data:image2/jpeg;base64," + source2;
                    
                }
            }

            var point = JSON.parse(data.pts);
            if(point != "")
            {
                paintPoints(point);  //TODO: DO NOT TOUCH
            }
            var matching = JSON.parse(data.match);

            if(matching != "")
            {
                SetMatching(matching);
            }
            // Paint Maching
            paint_matching = data.p_match
            if (paint_matching === "T") {
                paintMatching();
            }
}
        


// Request Animation Frame to remove the flickers
function update_image(){
    let canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');
    canvas.height = 240;
    canvas.width = 650;

    context.drawImage(image1, 0, 0,320, 240);
    context.drawImage(image2, 320, 0,320, 240);
}

function paintPoints(points_received)
{
    var point = [];

    for (var i = 0; i < points_received.length; i++)
    {
        point.x = points_received[i][0];
        point.y = points_received[i][1];
        point.z = points_received[i][2];
        point.r = points_received[i][3];
        point.g = points_received[i][4];
        point.b = points_received[i][5];
		addPoint(point);
    }
}

function getRandomColor() {
  var letters = '0123456789ABCDEF';
  var color = '#';
  for (var i = 0; i < 6; i++) {
    color += letters[Math.floor(Math.random() * 16)];
  }
  return color;
}

function line_matching_image(matching)
{
    let canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');


    context.beginPath();
    context.moveTo(matching.x1, matching.y1);
    context.lineTo(matching.x2 + 320, matching.y2);
    context.arc(matching.x1, matching.y1, 5, 0, 2 * Math.PI);
    context.arc(matching.x2 + 320, matching.y2, 5, 0, 2 * Math.PI);
    context.strokeStyle = matching.color;
    context.fillStyle = matching.color;
    context.fill();
    context.stroke();
}

function paintMatching()
{
    reset_matching()

    var matching = [];

    for (var i = 0; i < matching_history.length; i++)
    {
        matching.x1 = matching_history[i][0]/2;
        matching.y1 = matching_history[i][1]/2;
        matching.x2 = matching_history[i][2]/2;
        matching.y2 = matching_history[i][3]/2;
        matching.color = matching_history_color[i];

        line_matching_image(matching);
    }
    matching_history = []
    matching_history_color=[]
}

function SetMatching(matching_received)
{
    matching_history = matching_history.concat(matching_received);
    for (var i = 0; i < matching_received.length; i++)
    {
        matching_history_color = matching_history_color.concat(getRandomColor());
    }

}

function reset_matching() {
    let canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');

    // Clear the canvas
    context.clearRect(0, 0, canvas.width, canvas.height);

    // Redraw the images
    context.drawImage(image1, 0, 0,320, 240);
    context.drawImage(image2, 320, 0,320, 240);
}


export function reset_all() {
    let canvas = document.getElementById("gui_canvas"),
    context = canvas.getContext('2d');

    // Clear the canvas
    context.clearRect(0, 0, canvas.width, canvas.height);
}

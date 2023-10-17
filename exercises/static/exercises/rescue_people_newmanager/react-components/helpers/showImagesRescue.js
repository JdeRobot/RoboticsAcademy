// To decode the image string we will receive from server
function decode_utf8(s){
    return decodeURIComponent(escape(s))
}

let image = new Image();
let image_left = new Image();

export function drawImage (data){
    var canvas = document.getElementById("gui_canvas_right"),
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

export function drawLeftImage (data){
    var canvas_left = document.getElementById("gui_canvas_left"),
    context_left = canvas_left.getContext('2d')

    // For image object
    image_left.onload = function(){
        update_left_image();
    }

    // Request Animation Frame to remove the flickers
    function update_left_image(){
        window.requestAnimationFrame(update_left_image);
        context_left.drawImage(image_left, 0, 0);
    }
            
    // Parse the Image Data
    var image_data = JSON.parse(data.image_left),
        source = decode_utf8(image_data.image_left),
        shape = image_data.shape;
    
    if(source != ""){
        image_left.src = "data:image/jpeg;base64," + source;
        canvas_left.width = shape[1];
        canvas_left.height = shape[0];
    }   
}

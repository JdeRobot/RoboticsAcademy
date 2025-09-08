// To decode the image string we will receive from server
function decode_utf8(s) {
  return decodeURIComponent(escape(s));
}

let image_right = new Image();
let image_left = new Image();

export function drawImage(data) {
  var canvas = document.getElementById("gui_canvas_right");

  // Request Animation Frame to remove the flickers
  function decode_utf8(s) {
      return decodeURIComponent(escape(s))
  }

  // Parse the Image Data
  var image_data = JSON.parse(data.image_right),
    source = decode_utf8(image_data.image_right),
    shape = image_data.shape_right;

  if (source != "" && shape instanceof Array) {
    canvas.src = "data:image/jpeg;base64," + source;
    canvas.width = shape[1];
    canvas.height = shape[0];
  }
}

export function drawLeftImage(data) {
  var canvas = document.getElementById("gui_canvas_left");

  // Request Animation Frame to remove the flickers
  function decode_utf8(s) {
      return decodeURIComponent(escape(s))
  }

  // Parse the Image Data
  var image_data = JSON.parse(data.image_left),
    source = decode_utf8(image_data.image_left),
    shape = image_data.shape_left;

  if (source != "" && shape instanceof Array) {
    canvas.src = "data:image/jpeg;base64," + source;
    canvas.width = shape[1];
    canvas.height = shape[0];
  }
}

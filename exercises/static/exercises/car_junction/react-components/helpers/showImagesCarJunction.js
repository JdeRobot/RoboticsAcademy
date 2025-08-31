// To decode the image string we will receive from server
function decode_utf8(s) {
  return decodeURIComponent(escape(s));
}

let image_front = new Image();

export function drawImage(data) {
  var canvas = document.getElementById("gui_canvas_front");

  // Request Animation Frame to remove the flickers
  function decode_utf8(s) {
      return decodeURIComponent(escape(s))
  }

  // Parse the Image Data
  var image_data = JSON.parse(data.image_front),
    source = decode_utf8(image_data.image_front),
    shape = image_data.shape_front;

  if (source != "" && shape instanceof Array) {
    canvas.src = "data:image/jpeg;base64," + source;
    canvas.width = shape[1];
    canvas.height = shape[0];
  }
}

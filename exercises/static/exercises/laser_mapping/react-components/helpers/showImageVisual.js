// To decode the image string we will receive from server
function decode_utf8(s) {
  return decodeURIComponent(escape(s));
}

let image = new Image();

export function drawImage(data) {
  var canvas = document.getElementById("gui-canvas");

  // Request Animation Frame to remove the flickers
  function decode_utf8(s) {
      return decodeURIComponent(escape(s))
  }

  // Parse the Image Data
  var image_data = JSON.parse(data.user_map),
    source = decode_utf8(image_data.user_map),
    shape = image_data.shape;

  if (source != "" && shape instanceof Array) {
    canvas.src = "data:user_map/jpeg;base64," + source;
    canvas.width = shape[1];
    canvas.height = shape[0];
  }
}

export const updatePath = (trail, setPathCallback, height, width) => {
  var newPath = "M ";

  for (let index = 0; index < trail.length; index++) {
    const element = trail[index];
    var top  = element[0] * height;
    var left = element[1] * width;
    if (index === 0) {
      newPath += left.toString()+ "," + top.toString();
    }
    newPath += " L " + left.toString() + "," + top.toString();
  }

  if (trail.length > 0) {
    setPathCallback(newPath)
  }
}

export const addToPath = (x, y, trail) => {
  if (!trail.includes([x, y])) {
    trail.push([x, y]);
  }
}

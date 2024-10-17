export const updatePath = (trail, setPathCallback, height, width) => {
  var newPath = "M ";

  for (let index = 0; index < trail.length; index++) {
    const element = trail[index];
    var top  = element[1] * height;
    var left = element[0] * width;
    if (index === 0) {
      newPath += left.toString()+ "," + top.toString();
    }
    newPath += " L " + left.toString() + "," + top.toString();
  }

  if (trail.length > 0) {
    setPathCallback(newPath)
  }
}

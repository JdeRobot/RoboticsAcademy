export const getCarPose = (circuit, position) => {
  var scaleX, scaleY, offsetX, offsetY;
  let pose = position.substring( 1, position.length - 1);
  let pos = pose.split(",").map(function (item) {
    return parseFloat(item);
  });
  
  switch (circuit) {
    case "simple":
      scaleY = 1.25; offsetY = 77
			scaleX = -2.6; offsetX = 151
      break;
    case "montmelo":
      scaleY = 2.1; offsetY = 77
			scaleX = -1.3; offsetX = 151
      break;
    case "montreal":
			// scaleY = 0.6; offsetY = 76
      scaleY = 0.685; offsetY = 77
			scaleX = -0.48; offsetX = 151
      break;
    case "nürburgring":
      scaleY = 1.5; offsetY = 77
			scaleX = -1.495; offsetX = 151
      break;
    default:
      scaleY = 1.25; offsetY = 77
			scaleX = -2.6; offsetX = 151
      break;
  }

  var x = Math.round(pos[0]) * scaleX + offsetX;
  var y = Math.round(pos[1]) * scaleY + offsetY;

  return [x,y];
};
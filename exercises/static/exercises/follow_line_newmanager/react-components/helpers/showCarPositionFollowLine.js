const scaleX = 0.2;
const scaleY = 0.2;

export const getCarPose = (position) => {
  let pose = position.substring( 1, position.length - 1);
  let pos = pose.split(",").map(function (item) {
    return parseFloat(item);
  });

  var x = 16 + Math.round(pos[0]) * scaleX;
  var y = 16 + Math.round(pos[1]) * scaleY;

  return [x,y];
};
const scaleX = 1;
const scaleY = 1;

export const getCarPose = (position) => {
  let pose = position.substring( 1, position.length - 1);
  let pos = pose.split(",").map(function (item) {
    return parseFloat(item);
  });

  var x = Math.round(pos[0]) * scaleX;
  var y = Math.round(pos[1]) * scaleY;

  return [x,y];
};
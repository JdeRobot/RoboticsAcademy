// Maps a world (x, y) onto the aerial image of the simple circuit, same scale as follow_line
const SCALE = { scaleX: -2.6, offsetX: 151, scaleY: 1.25, offsetY: 77 };
// The obstacle avoidance world moves the simple circuit away from the origin
const CIRCUIT_ORIGIN = { x: -53.46, y: 11.5 };

export const getCarPose = (position) => {
  const { scaleX, offsetX, scaleY, offsetY } = SCALE;
  let pose = position.substring(1, position.length - 1);
  let pos = pose.split(",").map(function (item) {
    return parseFloat(item);
  });

  var x = Math.round(pos[0] - CIRCUIT_ORIGIN.x) * scaleX + offsetX;
  var y = Math.round(pos[1] - CIRCUIT_ORIGIN.y) * scaleY + offsetY;

  // Heading on the image, its axes are flipped and scaled unlike the world
  var yaw = pos.length > 2 ? pos[2] : 0;
  var angle =
    (Math.atan2(Math.sin(yaw) * scaleY, Math.cos(yaw) * scaleX) * 180) /
    Math.PI;

  return [x, y, angle];
};

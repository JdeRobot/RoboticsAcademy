// Maps a world (x, y) onto the aerial image of each circuit, same scales as follow_line
const SCALES = {
  default: { scaleX: -2.6, offsetX: 151, scaleY: 1.25, offsetY: 77 },
  montmelo: { scaleX: -1.3, offsetX: 151, scaleY: 2.1, offsetY: 77 },
  montreal: { scaleX: -0.48, offsetX: 151, scaleY: 0.685, offsetY: 77 },
};

export const getCarPose = (circuit, position) => {
  const { scaleX, offsetX, scaleY, offsetY } =
    SCALES[circuit] || SCALES.default;
  let pose = position.substring(1, position.length - 1);
  let pos = pose.split(",").map(function (item) {
    return parseFloat(item);
  });

  var x = Math.round(pos[0]) * scaleX + offsetX;
  var y = Math.round(pos[1]) * scaleY + offsetY;

  // Heading on the image, its axes are flipped and scaled unlike the world
  var yaw = pos.length > 2 ? pos[2] : 0;
  var angle =
    (Math.atan2(Math.sin(yaw) * scaleY, Math.cos(yaw) * scaleX) * 180) /
    Math.PI;

  return [x, y, angle];
};

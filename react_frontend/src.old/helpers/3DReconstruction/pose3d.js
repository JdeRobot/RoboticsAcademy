export function getYaw(qw, qx, qy, qz) {
  const rotateZa0 = 2.0 * (qx * qy + qw * qz);
  const rotateZa1 = qw * qw + qx * qx - qy * qy - qz * qz;
  if (rotateZa0 !== 0.0 && rotateZa1 !== 0.0) {
    return Math.atan2(rotateZa0, rotateZa1);
  }
  return 0.0;
}

export function getRoll(qw, qx, qy, qz) {
  const rotateXa0 = 2.0 * (qy * qz + qw * qx);
  const rotateXa1 = qw * qw - qx * qx - qy * qy + qz * qz;

  if (rotateXa0 !== 0.0 && rotateXa1 !== 0.0) {
    return Math.atan2(rotateXa0, rotateXa1);
  }
  return 0.0;
}
export function getPitch(qw, qx, qy, qz) {
  const rotateYa0 = -2.0 * (qx * qz - qw * qy);
  if (rotateYa0 >= 1.0) {
    return Math.PI / 2.0;
  } else if (rotateYa0 <= -1.0) {
    return -Math.PI / 2.0;
  } else {
    return Math.asin(rotateYa0);
  }
}

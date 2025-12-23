export const getCarPose = (circuit, position) => {

  let pose = position.substring(1, position.length - 1);
  let pos = pose.split(",").map(item => parseFloat(item));
  let x, y;

  switch (circuit) {

    case "default": {
      const scaleY = 1.25, offsetY = 77;
      const scaleX = -2.6, offsetX = 151;
      x = pos[0] * scaleX + offsetX;
      y = pos[1] * scaleY + offsetY;
      break;
    }

    case "montmelo": {
      const scaleY = 2.1, offsetY = 77;
      const scaleX = -1.3, offsetX = 151;
      x = pos[0] * scaleX + offsetX;
      y = pos[1] * scaleY + offsetY;
      break;
    }

    case "montreal": {
      const scaleY = 0.685, offsetY = 77;
      const scaleX = -0.48, offsetX = 151;
      x = pos[0] * scaleX + offsetX;
      y = pos[1] * scaleY + offsetY;
      break;
    }

    case "ngb": {
      const scaleY = 1.5, offsetY = 77;
      const scaleX = -1.495, offsetX = 151;
      x = pos[0] * scaleX + offsetX;
      y = pos[1] * scaleY + offsetY;
      break;
    }

    // case "monaco": {
    //   const a  =  0.70789644;
    //   const b  =  0.64807931;
    //   const c  =  1.31606339;
    //   const d  = -1.22986948;
    //   const tx = 146.68737022;
    //   const ty = 123.51318472;
    //   x = a * pos[0] + b * pos[1] + tx;
    //   y = c * pos[0] + d * pos[1] + ty;
    //   break;
    // }

    case "monaco": {
      const a  =  0.69947078;
      const b  =  0.68147198;
      const c  =  1.26589803;
      const d  = -1.22712405;
      const tx = 146.68737022;
      const ty = 123.51318472;
      x = a * pos[0] + b * pos[1] + tx;
      y = c * pos[0] + d * pos[1] + ty;
      break;
    }

    default: {
      const scaleY = 1.25, offsetY = 77;
      const scaleX = -2.6, offsetX = 151;
      x = pos[0] * scaleX + offsetX;
      y = pos[1] * scaleY + offsetY;
      break;
    }

  }
  return [Math.round(x), Math.round(y)];
};

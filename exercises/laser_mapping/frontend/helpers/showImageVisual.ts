export const updatePath = (
  trail: number[][],
  setPathCallback: (path: string) => void,
  height: number,
  width: number,
) => {
  var newPath = "M ";

  for (let index = 0; index < trail.length; index++) {
    const element = trail[index];
    var top = element[0] * height;
    var left = element[1] * width;
    if (index === 0) {
      newPath += left.toString() + "," + top.toString();
    }
    newPath += " L " + left.toString() + "," + top.toString();
  }

  if (trail.length > 0) {
    setPathCallback(newPath);
  }
};

export const addToPath = (x: number, y: number, trail: number[][]) => {
  if (!trail.includes([x, y])) {
    trail.push([x, y]);
  }
};

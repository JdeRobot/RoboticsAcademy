import { CIRCUITS_CHECKPOINTS } from "./checkpointsFollowLine";

let n = 0

export const getProgress = (world, position) => {
  let checkpoints = CIRCUITS_CHECKPOINTS[world];

  var x = Math.round(position[0]);
  var y = Math.round(position[1]);

  var d = Math.sqrt(
    Math.pow(checkpoints[n][1] - x, 2) + Math.pow(checkpoints[n][2] - y, 2)
  );
  var d1 = Math.sqrt(
    Math.pow(checkpoints[n][1] + 5 - x, 2) +
      Math.pow(checkpoints[n][2] + 5 - y, 2)
  );
  var d2 = Math.sqrt(
    Math.pow(checkpoints[n][1] - 5 - x, 2) +
      Math.pow(checkpoints[n][2] - 5 - y, 2)
  );
  var d3 = Math.sqrt(
    Math.pow(checkpoints[n][1] + 5 - x, 2) +
      Math.pow(checkpoints[n][2] - 5 - y, 2)
  );
  var d4 = Math.sqrt(
    Math.pow(checkpoints[n][1] - 5 - x, 2) +
      Math.pow(checkpoints[n][2] + 5 - y, 2)
  );

  var threshold = 5;
  if (
    d <= threshold ||
    d1 <= threshold ||
    d2 <= threshold ||
    d3 <= threshold ||
    d4 <= threshold
  ) {
    n = n + 1;

    if (n >= checkpoints.length) {
      return 0;
    }
  }

  var progress_bar = Math.round((n / checkpoints.length) * 100);

  if (n >= checkpoints.length && progress_bar === 0) {
    n = 0;
    return 100;
  } else {
    return progress_bar;
  }
};

export const resetProgress = () => {
  n = 0;
}
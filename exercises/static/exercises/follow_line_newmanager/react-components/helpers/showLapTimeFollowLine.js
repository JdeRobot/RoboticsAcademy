export const displayLapTime = (lap) => {
  let lapArray = lap.split(":")
  let secondsArray = lapArray[2].split(".")

  let s = Number(lapArray[0])*3600 + Number(lapArray[1])*60 + Number(secondsArray[0]);
  let ms = Number(secondsArray[1].slice(0,2))

  return `${s}.${ms.toString().padStart(2,0)}`
}

export const isLapFinished = (circuit, pose) => {
  var thresholdX, thresholdY;
  var x = pose[0]
  var y = pose[1]

  switch (circuit) {
    case "simple":
      thresholdX = [9, 15]
      thresholdY = [64, 66]
      break;
    case "montmelo":
      thresholdX = [112, 115]
      thresholdY = [7, 13]
      break;
    case "montreal":
      thresholdX = [245, 247]
      thresholdY = [14, 18]
      break;
    case "nürburgring":
      thresholdX = [262, 265]
      thresholdY = [132, 136]
      break;
    default:
      thresholdX = [9, 15]
      thresholdY = [64, 66]
      break;
  }

  console.log(x,y)
  console.log(thresholdX,thresholdY)

  return x > thresholdX[0] && x < thresholdX[1] && 
         y > thresholdY[0] && y < thresholdY[1];
}

export const isInTimeoutLap = (lapTime, timeout) => {
  var deltaLapFinished = Math.floor((Date.now() - lapTime)/1000);

  return lapTime !== null && deltaLapFinished < timeout
}

export const addLap = (oldLaps, time) => {
  var lapTime = displayLapTime(time).split(".");
  const newS = Number(lapTime[0]);
  const newMs = Number(lapTime[1]);

  var totalS = 0, totalMs = 0;
  for (let index = 0; index < oldLaps.length; index++) {
    var oldlapTime = oldLaps[index].split(".");
    totalS += Number(oldlapTime[0]);
    totalMs += Number(oldlapTime[1]);
  }

  const finalS = newS - totalS;
  const finalMs = newMs - totalMs;

  if (finalMs < 0) {
    finalS -= 1;
    finalMs += 100; 
  }

  return `${finalS}.${finalMs.toString().padStart(2,0)}`
}
export const displayLapTime = (circuitName, pose, lap) => {
  let lapArray = lap.split(":")
  let secondsArray = lapArray[2].split(".")

  let s = Number(lapArray[0])*3600 + Number(lapArray[1])*3600 + Number(secondsArray[0]);
  let ms = Number(secondsArray[1].slice(0,2))

  return `${s}.${ms.toString().padStart(2,0)}`
} 
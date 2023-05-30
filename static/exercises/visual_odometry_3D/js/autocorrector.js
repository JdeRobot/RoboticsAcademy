var scoreElement = document.getElementById("score-number");

function updateScore(number) {
  var actualScore = parseInt(scoreElement.innerText.split(":")[1]);
  var newScore = actualScore + number;
  scoreElement.innerText = "Score: " + newScore;
  console.log("scoreElement: ", scoreElement)
}

function autocorrector(truePosition, estimatedPosition) {
    // Euclidedistance
    var d = Math.sqrt(
        truePosition.x * estimatedPosition.x + 
        truePosition.y * estimatedPosition.y + 
        truePosition.z * estimatedPosition.z);
  
    var increment = 5 / (1 + Math.exp(-(Math.abs(d) - 10)));
    console.log("Increment: ", increment)
    updateScore(increment);
  }

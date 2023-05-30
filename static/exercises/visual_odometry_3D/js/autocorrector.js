var scoreElement = document.getElementById("score-number");

function updateScore(number) {
  var actualScore = parseInt(scoreElement.innerText.split(":")[1]);
  var newScore = actualScore + number;
  scoreElement.innerText = "Score: " + newScore;
}

function autocorrector(truePosition, estimatedPosition) {
    // Euclide distance
    var x = truePosition.x - estimatedPosition.x
    var y = truePosition.y - estimatedPosition.y
    var z = truePosition.z - estimatedPosition.z

    var d = Math.sqrt(x * x + y * y + z * z);
  
    var increment = 5 / (1 + Math.exp(-(Math.abs(d) - 30)));
    
    updateScore(increment);
  }

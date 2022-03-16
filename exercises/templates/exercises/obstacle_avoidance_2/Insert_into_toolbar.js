
var inn = "{% include 'medidorRTF.html' %}" +
       "{% include 'resetButton.html' %}" +
        "{% include 'loadButton.html' %}" +
        "{% include 'saveButton.html' %}" +
        "{% include 'loadIntoRobot.html' %}" +
		"{% include 'playButton.html' %}" +
		"{% include 'stopButton.html' %}";



document.getElementById('Control').innerHTML += inn;
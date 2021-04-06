document.write(
    `  
  


  <link rel="stylesheet" href="../../_components/assets/code/main.css">
  <link rel="stylesheet" href="../../_components/assets/code/split_style.css">
  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bootstrap@4.5.3/dist/css/bootstrap.min.css" integrity="sha384-TX8t27EcRE3e/ihU7zmQxVncDAy5uIKz4rEkgIXeMed4M0jlfIDPvg6uqKI2xXr2" crossorigin="anonymous">
</head>

<body>
  
<!-- Extends base.html to initiate the connection to the RADI -->
<iframe id="radiConnect" type="text/html" src="../../assets/base.html" width="100%" height="7%" name="data" frameBorder="0" scrolling="no"></iframe>
<script>
    var radiConect = document.getElementById('radiConnect')
    radiConect.contentWindow.postMessage('message', '*');
    window.onmessage = function(event){
        if (event.data == 'start') {
          console.log(event)
            startSim()
        }
    };
</script>
<div class="content">
<div class="split a">
<div id="Control" class="container" >
  <button  id="submit" type="button" onclick="submitCode()"><img src="./assets/img/submit.png"> Play </button>
  <button  id="stop" type="button" onclick="stopCode()"><img src="./assets/img/pause.png"> Stop</button>
  <button  id="save" type="button" onclick="saveCodeLocally()"><img src="./assets/img/upload.png"> Save</button>
  <input   type="file" id="real-file" hidden="hidden"/>
  <button  id="load" type="button"><img src="./assets/img/download.png"> Load</button>
  <button  id="reset" type="button" onclick="resetSim()"><img style="height:15px;" src="./assets/img/reset.svg"> Reset</button>
</div>
  
<li id="frecuency_control" style="list-style: none; display:block; text-align:center; font-weight:bold;" ><div id="Frequency" class="container">
<!-- Source http://thenewcode.com/757/Playing-With-The-HTML5-range-Slider-Input -->
<table>
<tr><strong>
<th>Name</th>
<th>Slider</th>
<th>Frequency Measure</th>
</strong>
</tr>
<tr>
<td>Brain</td>
<td><input type="range" min="1" max="30" value="12.5" id="fader"
step="0.1" oninput="codefrequencyUpdate(value)" class="frequency-slider"></td>
<td><output id="ideal_code_frequency">0</output> / <output for="fader" id="code_frequency">12.5</output></td>
<td></td>
</tr>
<tr>
<td>GUI</td>
<td><input type="range" min="1" max="30" value="12" id="fader"
step="0.5" oninput="guifrequencyUpdate(value)" class="frequency-slider"></td>
<td><output id="ideal_gui_frequency">0</output> / <output for="fader" id="gui_frequency">12.5</output></td>
</tr>
<tr>
<th >Debug-Level</th>
<td ><input  id="slide" type="range" min="1" max="4" steps="1" value="2" name="debug" class="debug-slider">
<ul class="debug-labels">
<th><p id="slidernum">2</p>
  <script type="text/javascript">
  var slide = document.getElementById('slide'),
  sliderDiv = document.getElementById("slidernum");
  
  slide.onchange = function() {
  sliderDiv.innerHTML = this.value;
  }
  </script>
</ul>

</td>
</th>
</tr>

</table>
</div>
</li>
  
  
<div id="code-control">
<!-- Code Editor -->
<!-- Along with buttons -->
<div id="code_container">
<div id="editor">from GUI import GUI
from HAL import HAL
# Enter sequential code!
while True:
# Enter iterative code!
</div>
</div>
</div>
</div>
  
  

     
    `
)
var checkpoints= [[1,12,42], [2,12,36], [3,12,34], [4,12,23], [5,14,16], [6,19,13],[7,31,12], [8,41,14], [9,43,15], [10,47,35],
                  [11,47,39],[12,51,45], [13,63,48], [14,91,48], [15,112,48], [16,150,48],[17,166,42], [18,167,32], [19,171,27], [20,199,18],
                  [21,220,19], [22,234,21], [23,253,27], [24,261,30], [25,268,34], [26,276,39],[27,279,42], [28,286,51], [29,288,55], [30,289,59],
                  [31,289,61],[32,284,77], [33,281,81], [34,280,82], [35,277,84], [36,259,95],[37,225,104], [38,219,105], [39,191,109], [40,181,111],
                  [41,167,113], [42,160,114], [43,145,116], [44,134,130], [45,134,134], [46,123,141],[47,111,141], [48,99,134], [49,80,116], [50,65,124],
                  [51,64,134],  [52,60,134], [53,36,137], [54,22,136], [55,12,129], [56,12,110],[57,12,100], [58,12,94],[59,12,84],[60,12,63]
];

var start_pos = [12,63]

var n = 0;
var point = 0;
var lap = 0;


const reset = document.getElementById("reset");

reset.addEventListener("click", function(){
  document.getElementById('porcentaje_bar').innerHTML = 0 +'%';
  document.getElementById('porcentaje_bar').style.width = 0+'%';
  n = 0;
  point = 0;
  lap = 0;
});

function ResetEvaluator() {
  document.getElementById('porcentaje_bar').innerHTML = 0 +'%';
  document.getElementById('porcentaje_bar').style.width = 0+'%';
  n = 0;
  point = 0;
  lap = 0;
}

//gui.js data 
function Evaluator(content){
   var x = Math.round(content[0])
   var y = Math.round(content[1])
   //console.log(x,y)
   //console.log(checkpoints[n][1],checkpoints[n][2])
   var d = Math.sqrt(Math.pow((checkpoints[n][1]-x), 2)+Math.pow((checkpoints[n][2]-y), 2));
   var d1 = Math.sqrt(Math.pow((checkpoints[n][1]+5-x), 2)+Math.pow((checkpoints[n][2]+5-y), 2));
   var d2 = Math.sqrt(Math.pow((checkpoints[n][1]-5-x), 2)+Math.pow((checkpoints[n][2]-5-y), 2));
   var d3 = Math.sqrt(Math.pow((checkpoints[n][1]+5-x), 2)+Math.pow((checkpoints[n][2]-5-y), 2));
   var d4 = Math.sqrt(Math.pow((checkpoints[n][1]-5-x), 2)+Math.pow((checkpoints[n][2]+5-y), 2));
   //console.log(d)
   //console.log(n)
   if (d <= 10 || d1 <= 15 || d2 <= 15 || d3 <= 15 || d4 <= 15){
       n+=1
     if(n >= 60){
       lap+= 1
       console.log("vuelta finalizada", lap)
       var lap_time_display = document.getElementById("lap_time").textContent;
       document.getElementById('lap').innerHTML += 'Lap ' + lap + ' ' + lap_time_display+'<br>';
       document.getElementById('porcentaje_bar').innerHTML = 100 +'%';
       document.getElementById('porcentaje_bar').style.width = 100+'%';

     }
   }
   var porcentaje= document.getElementById('evaluator')
   var progress_bar = Math.round(n/checkpoints.length*100)
//  porcentaje.innerHTML= 'Pocentaje' + Math.round(n/checkpoints.length*100) +'%'

  if (n!=60){
    document.getElementById('porcentaje_bar').innerHTML = progress_bar +'%';
    document.getElementById('porcentaje_bar').style.width = progress_bar+'%';
  }else{
    n=0;
  }

 }

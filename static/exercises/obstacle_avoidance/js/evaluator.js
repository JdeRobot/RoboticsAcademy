let laser_puntuation= [];
let mean = 0;
let border_A = []
let border_B = []

var penalty=3
var exercise_score=[]

//W width  & H height
function Create_Border(array, W, H){
    //cartesian coordinates
    var step = H/60
    var step2 = W*2/60
    var n= 0;
    var n2= 60;
    for (let z = 0; z <180; z++) {
        if (z < 60) {
            x= W
            y= step*z
        }else if(z>=120){
 	    x= -W
            y= step*n2
	    n2=n2-1
        }else{
	    x= W-(step2*n)
	    y= H
            n=n+1
        }

        //polar coordinates
        var r=Math.sqrt((Math.pow(x, 2)+Math.pow(y, 2)));
        var angle=Math.atan(y/x) ;

        if(angle < 0){
            angle= angle + Math.PI;
        }

        var array_polar=[r,angle]
        //var array_cartesian=[x,y]

        array.push(array_polar);
	//array.push(array_cartesian)
    }
}

//Borders in polar coordinates
Create_Border(border_A,1,10)
Create_Border(border_B,10,25)

//console.log(border_A)
//console.log(border_B)


//Evaluator obstacle avoidance, laser security.
function Evaluator(laser){
    //console.log("Laser",laser);
    for (let i = 0; i < laser.length; i++) {
	if(laser[i][0] > 150){
		laser_puntuation[i]= 1;
	}else{
		//Zone A 1 point
		if (laser[i][0]< border_A[i][0] && laser[i][1]< border_A[i][1]) {
		    laser_puntuation[i]= 1;
		}
		//Zone B 5 points
		else if (laser[i][0]< border_B[i][0] && laser[i][1]< border_B[i][1] ){
		    laser_puntuation[i]= 5;
		//Zone C 10 points
		}else {
		    laser_puntuation[i]= 10;
		}
	}
    }

    for (let j=0; j < laser_puntuation.length; j++) {
        mean = mean + laser_puntuation[j];
    }

    var count = 0;
    for(var k = 0; k < laser_puntuation.length; ++k){
        if(laser_puntuation[k] == 1)
            count++;
    }

    //console.log(count);
    //console.log(laser_puntuation)

    mean = mean/laser_puntuation.length;

    //console.log(mean);

    //10% laser with bad puntuation
    if(count > 18){
        mean= mean - penalty;
    }
    if(count > 60){
        mean= 0;
    }
    if(mean> 10){
        mean = 10;
    }
    if(mean <0){
        mean = 0;
    }
}



window.onload = function Security() {

var ctx = document.getElementById('myChart').getContext('2d');

var chart = new Chart(ctx, {
    // The type of chart we want to create
    type: 'line',
    backgroundColor: "Green",

    // The data for our dataset
    data: {
        labels: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30],
        datasets: [{
            label: 'Security Evaluator',
            data: [],
	    backgroundColor: 'rgb(0, 0,0 )',
	    fill:false
        }],

    },

    // Configuration options go here
    options:{
		title: {
          	  display: true,
          	  text: 'Security Evaluator'
        	},
		legend: {
    			display: false
		},
		scales:{
		 yAxes: [{
			ticks:{
				min:0,
				max:10,
			}}
			]
		},

	}
});


var updateInterval = 1000;
var time = 0

function updateChart() {


     if (time >30){
	 var thirty =[30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30]

	 var r = [];

	 for(i = 0; i < chart.data.labels.length; i++){
             r[i] =chart.data.labels[i]+thirty[i];
	     chart.data.datasets.forEach((dataset) => {
          dataset.data.pop();

     });
         }
	 chart.data.labels =[]
         chart.data.labels= r
	 chart.data.datasets.forEach((dataset) => {
          dataset.data.pop();

     	});
	time = 0

     }
     chart.data.datasets.forEach((dataset) => {
          dataset.data.push(mean);
     });
     time++


	 if(mean <= 5){
                document.getElementById('myChart').style.backgroundColor="rgba(255, 85, 85, 0.2)"
         }else{
                document.getElementById('myChart').style.backgroundColor="rgba(0, 255, 0, 0.2)"
         }

     chart.update();


	exercise_score.push(mean)

        //Global Score
	var score= 0;
	for (let k=0; k < exercise_score.length; k++) {
	    score = score + exercise_score[k];
	}

	score = score/exercise_score.length;
	document.getElementById('score').innerHTML = Math.round(Number.parseFloat(score).toFixed(2)*10);

    }


    var IntervalId;
    const play = document.getElementById("submit");
    play.addEventListener("click", function(){

        IntervalId = setInterval(function(){updateChart();}, updateInterval);

    });

    const efficacy = document.getElementById("efficacy");
    efficacy.addEventListener("click", function(){
        var second_click = $('#efficacy').data('clicks');
        if(second_click == "True"){
          IntervalId = setInterval(function(){updateChart();}, updateInterval);
        }else{
          if (IntervalId ){
        	clearInterval(IntervalId);
                }
        }
    });

    const stop = document.getElementById("stop");
    stop.addEventListener("click", function(){
	if (IntervalId ){
	clearInterval(IntervalId);
        }
    });

    const reset = document.getElementById("reset");
    reset.addEventListener("click", function(){
	time = 0;
	exercise_score=[]
	if (IntervalId ){
	clearInterval(IntervalId);
        }

	for(i = 0; i < chart.data.labels.length; i++){
	    chart.data.datasets.forEach((dataset) => {
                dataset.data.pop();
            })
        }

	var ctx = document.getElementById('myChart').getContext('2d');

	chart = new Chart(ctx, {
         // The type of chart we want to create
           type: 'line',
           backgroundColor: "Green",

           // The data for our dataset
            data: {
                labels: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30],
                datasets: [{
                         label: 'Security Evaluator',
                         data: [],
	                 backgroundColor: 'rgb(0, 0,0 )',
	                 fill:false
                }],
           },

           // Configuration options go here
           options:{
		title: {
          	  display: true,
          	  text: 'Security Evaluator'
        	},
		legend: {
    			display: false
		},
		scales:{
		 yAxes: [{
			ticks:{
				min:0,
				max:10,
			}}
			]
		},
           }
        });

    })

}

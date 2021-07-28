var draw = undefined,
    dist = 0,
    relayout_flag = false;

// Measure each point at an interval of 500 ms
var interval = 500;
// Evaluate for maximum of time 2 mins that is 120 secs
// 120 secs = 240 measuring points
var max_points = 240;
// Evaluate for minimum of 30 secs
// 30 secs = 60 measuring points
var min_points = 60;
// Each time the evaluate button is pressed, reduce time by 15 secs
// 15 secs = 30 measuring points
var reduce_points = 30;
// set reduce_points_flag to True after the first press of Evaluate button
var reduce_points_flag = false;

function calculate_score(dist, ready) {
    if (ready == "true") {
        if (dist)
    }
}

// Function for graph
function evaluator() {
    // stop existing plot if any
    clearInterval(draw);

    // start reducing points from the second press of Evaluate button
    if (max_points > min_points && reduce_points_flag == true) {
        max_points = max_points - reduce_points;
    }

    var xtickvals = [];
    var xticktext = [];
    for (let i = 0; i <= max_points; i+=reduce_points) {
        console.log(i*interval/1000)
        xtickvals.push(i);
        xticktext.push(i*interval/1000)
    }

    // https://plotly.com/javascript/reference/layout/
    data = {
        data : [{ y: [dist] }],
        layout : {
            title : "Drone Cat Mouse",
            width : 800, 
            height : 300, 
            plot_bgcolor : "#cfffcf",
            xaxis : {
                range : [0, max_points], 
                title : "Time (in seconds)", 
                tickvals : xtickvals,
                ticktext : xticktext
            }, 
            yaxis : {
                range : [0, 6], 
                title : "Distance", 
                tickvals : [1,2,3,4,5,6]
            }
        }
    };

    Plotly.newPlot("eval", data)
    var cnt = 0;

    draw = setInterval(function() {
        Plotly.extendTraces("eval", { y:[[dist]]}, [0]);
        cnt++;
        if (cnt >= max_points) {
            clearInterval(draw);
        }
        // change background color of the plot to red when distance > 4
        if(dist > 4) {
            Plotly.relayout("eval", {plot_bgcolor: "#ffcccb"});
            relayout_flag = true;
        } else if (dist <= 4 && relayout_flag == true ) {
            Plotly.relayout("eval", {plot_bgcolor: "#cfffcf"});
            relayout_flag = false;
        }
    },interval);

    reduce_points_flag = true;
}
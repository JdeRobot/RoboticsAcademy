var score = 0.0,
    dist_value = 0.0,
    dist_ready = "false"
    relayout_plot = false;

// Function to calculate and display score
function calculate_score() {
    if (dist_ready == "true") {
        if (dist_value < 2.0) score = score + 0.5;
        else if (dist_value >= 2.0 && dist_value < 3.0) score = score + 0.25;
        else if (dist_value >= 3.0 && dist_value < 4.0) score = score + 0.12;
        
        // set maximum score 100
        if (score >= 100.0) score = 100.0;

        // round score upto 2 decimals
        score = Math.round(score*100)/100;
        document.querySelector('#score').value = score;
    }
}

// Function to start evaluation
function startevaluate() {
    var eval_button = document.getElementById("eval_button");
    eval_button.disabled = true;
    eval_button.style.opacity = "0.4";
    eval_button.style.cursor = "not-allowed";

    // reset score and dist
    score = 0.0;
    dist = 0.0;

    // submit code
    check();

    // stop existing plot if any
    var eval = undefined;
    clearInterval(eval);

    // https://plotly.com/javascript/reference/layout/
    data = {
        data : [{ y: [dist_value] }],
        layout : {
            title : "Drone Cat Mouse",
            width : 800, 
            height : 300, 
            plot_bgcolor : "#cfffcf",
            xaxis : {
                range : [0, 240], 
                title : "Time (in seconds)", 
                tickvals : [0,30,60,90,120,150,180,210,240],
                ticktext : [0,15,30,45,60,75,90,105,120]
            }, 
            yaxis : {
                range : [0, 8], 
                title : "Distance", 
                tickvals : [1,2,3,4,5,6,7,8]
            }
        }
    };

    Plotly.newPlot("eval", data)
    var cnt = 0;

    eval = setInterval(function() {
        Plotly.extendTraces("eval", { y:[[dist_value]]}, [0]);
        cnt++;

        if (cnt >= 240) {
            clearInterval(eval);
            eval_button.disabled = false;
            eval_button.style.opacity = "1.0";
            eval_button.style.cursor = "default";
        }

        // change background color of the plot to red when distance > 4
        if(dist_value > 4) {
            Plotly.relayout("eval", {plot_bgcolor: "#ffcccb"});
            relayout_plot = true;
        } else if (dist_value <= 4 && relayout_plot == true ) {
            Plotly.relayout("eval", {plot_bgcolor: "#cfffcf"});
            relayout_plot = false;
        }

        // calculate and display score
        calculate_score();
    },500);
}
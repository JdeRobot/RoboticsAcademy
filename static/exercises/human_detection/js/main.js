
$('#code-menu').on('change', function() {
    if (this.value === "video") {
      $("#video-upload").css("display", "flex");
      $("#Video_Infer").css("display", "inline-block");
      $("#Live_Infer").css("display", "none");
      $("#benchmark").css("display", "none");
      $("#visualizer").css("display", "none")
      $("#stop_code").css("display", "inline-block");

    }
    else if (this.value === "live"){

        $("#Video_Infer").css("display", "none");
        $("#Live_Infer").css("display", "inline-block");
        $("#video-upload").css("display", "none");
        $("#benchmark").css("display", "none");        
        $("#visualizer").css("display", "none")
        $("#stop_code").css("display", "inline-block");
    } 
    else if (this.value === "bench"){
        $("#Video_Infer").css("display", "none");
        $("#video-upload").css("display", "none");
        $("#Live_Infer").css("display", "none");
        $("#benchmark").css("display", "inline-block");
        $("#visualizer").css("display", "none")
        $("#stop_code").css("display", "inline-block");
    } 
    else if (this.value === "visual"){
        $("#Video_Infer").css("display", "none");
        $("#video-upload").css("display", "none");
        $("#Live_Infer").css("display", "none");
        $("#benchmark").css("display", "none");
        $("#visualizer").css("display", "inline-block")
        $("#stop_code").css("display", "inline-block");
    } 
    else {

      $("#video-upload").css("display", "none");
      $("#Video_Infer").css("display", "none");
      $("#Live_Infer").css("display", "none");
      $("#benchmark").css("display", "none");
      $("#visualizer").css("display", "none")
      $("#stop_code").css("display", "none");
    }
  });
